from fastapi import FastAPI
from pydantic import BaseModel
from typing import List
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math

app = FastAPI()


class Point(BaseModel):
    name: str
    lat: float
    lon: float
    time_start: int  # начало окна (в минутах)
    time_end: int    # конец окна (в минутах)
    ice_kg: int
    raw_ice_kg: int = 0
    container_ice_kg: int = 0


class RouteRequest(BaseModel):
    warehouse: Point
    orders: List[Point]

def travel_time(p1, p2):
    dist_km = math.sqrt((p1.lat - p2.lat) ** 2 + (p1.lon - p2.lon) ** 2) * 100
    speed_kmh = 30 # Средняя скорость движения по городу
    return int((dist_km / speed_kmh) * 60)

def distance(p1, p2):
    """
    Приближённое расстояние между двумя точками (км)
    """
    return math.sqrt(
        (p1.lat - p2.lat) ** 2 +
        (p1.lon - p2.lon) ** 2
    )

@app.post("/build-route")
def build_route(data: RouteRequest):
    MAX_TOTAL_KG = 270
    RAW_ICE_TIME_LIMIT = 50  # минут

    warehouse = data.warehouse
    orders = data.orders

    points = [warehouse] + orders
    size = len(points)

    # --- разделение льда (Логика из ШАГА 3.3 и 3.4) ---
    for o in orders:
        if o.ice_kg >= 100:
            o.raw_ice_kg = o.ice_kg
            o.container_ice_kg = 0
        else:
            o.raw_ice_kg = 0
            o.container_ice_kg = o.ice_kg

    # Веса для Capacity Dimension (ШАГ 3.1)
    # Склад загружает всё, заказы выгружают
    warehouse.raw_ice_kg = sum(o.raw_ice_kg for o in orders)
    warehouse.container_ice_kg = sum(o.container_ice_kg for o in orders)
    
    # Для OR-Tools используем отрицательные значения для выгрузки (demand)
    # Но в этой реализации мы используем UnaryTransitCallback, который обычно работает с положительным спросом
    # Пересчитаем для классической модели: склад 0, заказы +вес
    
    dist_matrix = [
        [distance(points[i], points[j]) for j in range(size)]
        for i in range(size)
    ]

    manager = pywrapcp.RoutingIndexManager(size, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    # ---------- TIME (ШАГ 2) ----------
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return travel_time(points[from_node], points[to_node])

    time_cb = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(time_cb, 1440, 1440, False, "Time")
    time_dimension = routing.GetDimensionOrDie("Time")

    # ---------- CAPACITY (ШАГ 3.1) ----------
    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)
        if node == 0: return 0
        return points[node].ice_kg

    demand_cb = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimension(demand_cb, 0, MAX_TOTAL_KG, True, "Capacity")

    # ---------- TIME WINDOWS & RAW ICE LIMIT (ШАГ 3.4) ----------
    for i, point in enumerate(points):
        index = manager.NodeToIndex(i)
        
        if i == 0:
            time_dimension.CumulVar(index).SetRange(0, 1440)
            continue

        start = int(point.time_start)
        end = int(point.time_end)

        # Если есть сырой лед (>= 100кг), ограничиваем время доставки 50 минутами
        if point.raw_ice_kg > 0:
            end = min(end, RAW_ICE_TIME_LIMIT)

        if start > end:
            return {"error": f"Заказ '{point.name}' невозможен (лед растает раньше начала окна или окно некорректно)"}

        time_dimension.CumulVar(index).SetRange(start, end)

    # ---------- COST ----------
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(dist_matrix[from_node][to_node] * 1000)

    dist_cb = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_cb)

    # ---------- SOLVER ----------
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_params.time_limit.seconds = 5

    solution = routing.SolveWithParameters(search_params)

    if not solution:
        return {"error": "Route not found"}

    # ---------- RESULT (ШАГ 3.2) ----------
    index = routing.Start(0)
    route = []
    total_weight = 0

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        arrival = solution.Value(time_dimension.CumulVar(index))
        point_data = points[node]
        route.append({
            "point": point_data.name,
            "arrival_minute": arrival,
            "ice_kg": point_data.ice_kg,
            "type": "raw" if point_data.raw_ice_kg > 0 else "container"
        })
        index = solution.Value(routing.NextVar(index))

    # Финальное возвращение на склад
    node = manager.IndexToNode(index)
    arrival = solution.Value(time_dimension.CumulVar(index))
    route.append({
        "point": points[node].name,
        "arrival_minute": arrival,
        "type": "finish"
    })

    return {"route": route, "total_weight_kg": sum(o.ice_kg for o in orders)}
