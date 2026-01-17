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

    warehouse = data.warehouse
    orders = data.orders

    points = [warehouse] + orders
    size = len(points)

    # --- разделение льда ---

    # 1️ склад загружает весь raw лёд
    warehouse.raw_ice_kg = sum(
        o.ice_kg for o in orders if o.ice_kg >= 100
    )

    warehouse.container_ice_kg = sum(
        o.ice_kg for o in orders if o.ice_kg < 100
    )

    # 2️ заказы ВЫГРУЖАЮТ лёд
    for o in orders:
        if o.ice_kg >= 100:
            o.raw_ice_kg = -o.ice_kg
            o.container_ice_kg = 0
        else:
            o.raw_ice_kg = 0
            o.container_ice_kg = -o.ice_kg

    dist_matrix = [
        [distance(points[i], points[j]) for j in range(size)]
        for i in range(size)
    ]

    manager = pywrapcp.RoutingIndexManager(size, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    # ---------- TIME ----------
    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return travel_time(points[from_node], points[to_node])

    time_cb = routing.RegisterTransitCallback(time_callback)

    routing.AddDimension(
        time_cb,
        0,
        1440,
        False,
        "Time"
    )

    time_dimension = routing.GetDimensionOrDie("Time")

    RAW_ICE_TIME_LIMIT = 50  # минут

    def raw_ice_time_callback(from_index, to_index):
        # сколько raw льда В МАШИНЕ до выезда
        raw_load = solution.Value(
            raw_load_dimension.CumulVar(from_index)
        )

        if raw_load > 0:
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return travel_time(points[from_node], points[to_node])

        return 0

    raw_time_cb = routing.RegisterTransitCallback(raw_ice_time_callback)

    routing.AddDimension(
        raw_time_cb,
        0,                    # без ожиданий
        RAW_ICE_TIME_LIMIT,   # ⏱️ максимум 50 минут
        True,                 # старт = 0
        "RawIceTime"
    )

    # Стартуем строго в момент 0
    start_index = manager.NodeToIndex(0)
    time_dimension.CumulVar(start_index).SetValue(0)
    RAW_ICE_TIME_LIMIT = 50  # минут

    # ---------- CAPACITY ----------
    def raw_load_callback(from_index):
        node = manager.IndexToNode(from_index)
        return points[node].raw_ice_kg

    raw_load_cb = routing.RegisterUnaryTransitCallback(raw_load_callback)

    routing.AddDimension(
        raw_load_cb,
        0,
        MAX_TOTAL_KG,
        True,
        "RawLoad"
    )
    raw_load_dimension = routing.GetDimensionOrDie("RawLoad")

    # ---------- TIME WINDOWS (ТОЛЬКО ДЛЯ ЗАКАЗОВ) ----------
    for i, point in enumerate(points):
        if i == 0:
            continue

        index = manager.NodeToIndex(i)

        # обычные заказы
        start = point.time_start
        end = point.time_end

        # ❄️ RAW ICE — жесткое ограничение
        if point.ice_kg >= 100:
            end = min(end, RAW_ICE_TIME_LIMIT)

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

    # ---------- RESULT ----------
    index = routing.Start(0)
    route = []

    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        arrival = solution.Value(time_dimension.CumulVar(index))
        route.append({
            "point": points[node].name,
            "arrival_minute": arrival
        })
        index = solution.Value(routing.NextVar(index))

    # 🔥 ЯВНО добавляем возврат на склад
    arrival = solution.Value(time_dimension.CumulVar(index))
    route.append({
        "point": warehouse.name,
        "arrival_minute": arrival
    })

    return {"route": route}
