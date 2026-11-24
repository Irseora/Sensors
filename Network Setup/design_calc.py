#!/usr/bin/env python3
"""Assignment 10C – cost / bandwidth / compute calculator for a two-level in-vehicle network.

Usage:  python3 design_calc.py my_design.json
Inputs (same folder): sensors_and_nodes.csv, link_types.csv, compute_options.csv
Prices marked 'assumption' in the CSV files are teaching values set by the instructor, not market quotes.
"""
import csv, json, sys, math, os
HERE = os.path.dirname(os.path.abspath(__file__))

def load(name):
    with open(os.path.join(HERE, name), newline="") as f:
        return list(csv.DictReader(f))

NODES = {r["id"]: r for r in load("sensors_and_nodes.csv")}
LINKS = {r["link_type"]: r for r in load("link_types.csv")}
COMP = {r["item"]: r for r in load("compute_options.csv")}
BUS_TYPES = {"LIN", "CAN FD", "10BASE-T1S"}
BUS_MAX_NODES = {"LIN": 16, "CAN FD": 16, "10BASE-T1S": 8}   # incl. the zone port; CAN FD limit = course rule
SMART_ITEM = "Smart-camera processor option (per camera)"
SWITCH_ITEM = "Ethernet switch port (in zone or central)"

def xy(i): return float(NODES[i]["x_m"]), float(NODES[i]["y_m"])
def manh(a, b):
    (x1, y1), (x2, y2) = xy(a), xy(b)
    return abs(x1 - x2) + abs(y1 - y2)

def eucl(a, b):
    (x1, y1), (x2, y2) = xy(a), xy(b)
    return math.hypot(x1 - x2, y1 - y2)

def _path_cost(p):
    return (sum(manh(p[k], p[k + 1]) for k in range(len(p) - 1)),
            sum(eucl(p[k], p[k + 1]) for k in range(len(p) - 1)))

def chain_order(start, members):
    """Order of the nodes along one bus segment (LIN, CAN FD, 10BASE-T1S).
    These are linear buses: the cable runs from one end node to the other and the
    zone port / ECU may sit anywhere on it (not necessarily at an end).
    Shortest such line: nearest-neighbour start from every node, then 2-opt
    (segment reversal, ends free); ties broken by straight-line length."""
    nodes = [start] + list(members)
    best = None
    for s0 in nodes:
        left, path = [n for n in nodes if n != s0], [s0]
        while left:
            nxt = min(left, key=lambda m: (manh(path[-1], m), eucl(path[-1], m)))
            path.append(nxt); left.remove(nxt)
        cost = _path_cost(path); improved = True
        while improved:
            improved = False
            for i in range(len(path) - 1):
                for j in range(i + 1, len(path)):
                    cand = path[:i] + path[i:j + 1][::-1] + path[j + 1:]
                    c = _path_cost(cand)
                    if c[0] < cost[0] - 1e-9 or (abs(c[0] - cost[0]) < 1e-9 and c[1] < cost[1] - 1e-9):
                        path, cost, improved = cand, c, True
        if best is None or cost < best[1]:
            best = (path, cost)
    return best[0]

def chain_length(start, members):
    p = chain_order(start, members)
    return sum(manh(p[k], p[k + 1]) for k in range(len(p) - 1))

def main(path):
    d = json.load(open(path))
    rf, head = d.get("routing_factor", 1.3), d.get("headroom", 0.30)
    util = d.get("compute_utilisation", 0.5)
    fusion = d.get("central_fusion_TOPS", 20.0)
    errors, rows = [], []
    cost = {"cables": 0.0, "link ends (PHY/transceiver/connector)": 0.0, "switch ports": 0.0,
            "zone controllers": 0.0, "smart sensors": 0.0, "central compute": 0.0}
    mass_g, power_W = 0.0, 0.0
    tops_need = {"CC": fusion}; uplink = {}; zone_load = {}
    zones = d.get("zones", [])
    if len(zones) == 0:
        print("Single-level design: every node is wired directly to CC (no zone controllers).\n")
    elif not 2 <= len(zones) <= 4: errors.append(f"choose 0 or 2-4 zone sites, not {len(zones)}")
    for z in zones:
        if NODES.get(z, {}).get("class") != "zone_site": errors.append(f"{z} is not a zone site")
        kind = d.get("zone_compute", {}).get(z, "base")
        item = "Zone controller base (MCU, power, eFuses, housing)" if kind == "base" else "Zone controller + vision accelerator option"
        cost["zone controllers"] += float(COMP[item]["price_USD"]); power_W += float(COMP[item]["power_max_W"])
        tops_need[z] = 0.0; uplink[z] = 0.0
    groups = {}
    for nid, n in d["nodes"].items():
        if nid not in NODES: errors.append(f"unknown node {nid}"); continue
        att, lt, proc = n["attach"], n["link"], n.get("processing", "none")
        cls = NODES[nid]["class"]
        raw, done, tops = float(NODES[nid]["raw_output_Mbit_s"]), float(NODES[nid]["processed_output_Mbit_s"]), float(NODES[nid]["processing_TOPS"])
        if att != "CC" and att not in zones: errors.append(f"{nid}: attached to {att}, which is not a chosen zone"); continue
        if lt not in LINKS: errors.append(f"{nid}: unknown link type {lt}"); continue
        if cls in ("camera", "lidar", "radar") and proc not in ("sensor", "zone", "central"):
            errors.append(f"{nid}: processing must be sensor, zone or central"); continue
        if proc == "sensor":
            rate = done
            if cls in ("camera", "lidar"):
                cost["smart sensors"] += float(COMP[SMART_ITEM]["price_USD"]); power_W += float(COMP[SMART_ITEM]["power_max_W"])
                if tops > float(COMP[SMART_ITEM]["peak_TOPS_int8"]) * 1.0 + 1e-9 and cls == "lidar":
                    errors.append(f"{nid}: in-sensor processor (5 TOPS) too small for {tops} TOPS")
        elif proc == "zone":
            if att == "CC": errors.append(f"{nid}: processing 'zone' but attached to CC"); continue
            rate = raw; tops_need[att] += tops; uplink[att] += done
        else:
            rate = raw if cls in ("camera", "lidar", "radar") else done
            tops_need["CC"] += tops if cls in ("camera", "lidar", "radar") else 0
            if att != "CC": uplink[att] += rate
        if att != "CC" and proc != "zone": pass
        if lt in BUS_TYPES:
            groups.setdefault((att, lt, n.get("segment", 1)), []).append((nid, rate))
        else:
            L = manh(nid, att) * rf
            cap = float(LINKS[lt]["capacity_Mbit_s"])
            ok = rate <= cap * (1 - head)
            if not ok: errors.append(f"{nid}: {rate:.1f} Mbit/s exceeds {lt} with {int(head*100)} % headroom")
            cost["cables"] += L * float(LINKS[lt]["cable_USD_per_m"]); mass_g += L * float(LINKS[lt]["cable_g_per_m"])
            cost["link ends (PHY/transceiver/connector)"] += 2 * float(LINKS[lt]["USD_per_link_end"])
            if "BASE-T1" in lt: cost["switch ports"] += float(COMP[SWITCH_ITEM]["price_USD"])
            rows.append((nid, att, lt, rate, cap, L))
    bus_orders = []
    for (att, lt, seg), mem in groups.items():
        ids = [m for m, _ in mem]; load_ = sum(r for _, r in mem)
        L = chain_length(att, ids) * rf; cap = float(LINKS[lt]["capacity_Mbit_s"])
        if len(ids) + 1 > BUS_MAX_NODES[lt]: errors.append(f"{att}/{lt}/segment {seg}: {len(ids)+1} nodes > {BUS_MAX_NODES[lt]}")
        if load_ > cap * (1 - head): errors.append(f"{att}/{lt}/segment {seg}: {load_:.2f} Mbit/s exceeds capacity with headroom")
        cost["cables"] += L * float(LINKS[lt]["cable_USD_per_m"]); mass_g += L * float(LINKS[lt]["cable_g_per_m"])
        cost["link ends (PHY/transceiver/connector)"] += (len(ids) + 1) * float(LINKS[lt]["USD_per_link_end"])
        if lt == "10BASE-T1S": cost["switch ports"] += float(COMP[SWITCH_ITEM]["price_USD"])
        rows.append((f"bus[{','.join(ids)}]", att, lt, load_, cap, L))
        bus_orders.append(f"{att} {lt} segment {seg}: " + " - ".join(chain_order(att, ids)) + f"  ({L:.2f} m)")
    for z in zones:
        lt = d.get("backbone", {})[z]; cap = float(LINKS[lt]["capacity_Mbit_s"])
        L = manh(z, "CC") * rf
        if uplink[z] > cap * (1 - head): errors.append(f"backbone {z}: {uplink[z]:.0f} Mbit/s exceeds {lt} with headroom")
        cost["cables"] += L * float(LINKS[lt]["cable_USD_per_m"]); mass_g += L * float(LINKS[lt]["cable_g_per_m"])
        cost["link ends (PHY/transceiver/connector)"] += 2 * float(LINKS[lt]["USD_per_link_end"])
        cost["switch ports"] += 2 * float(COMP[SWITCH_ITEM]["price_USD"])
        rows.append((f"backbone {z}", "CC", lt, uplink[z], cap, L))
        kind = d.get("zone_compute", {}).get(z, "base")
        ztops = 0 if kind == "base" else float(COMP["Zone controller + vision accelerator option"]["peak_TOPS_int8"])
        if tops_need[z] > ztops * util + 1e-9: errors.append(f"zone {z}: needs {tops_need[z]} TOPS, has {ztops}×{util} usable")
    have = 0.0
    for m in d["central_compute"]:
        have += float(COMP[m]["peak_TOPS_int8"]); cost["central compute"] += float(COMP[m]["price_USD"]); power_W += float(COMP[m]["power_max_W"])
    if tops_need["CC"] > have * util + 1e-9: errors.append(f"central: needs {tops_need['CC']:.0f} TOPS, has {have:.0f}×{util} usable")
    # rule R: redundancy (fail-operational) - computers with AI capacity: central modules + vision zone controllers
    red_need = d.get("redundancy_TOPS", 20.0)
    ai = [float(COMP[m]["peak_TOPS_int8"]) for m in d["central_compute"]]
    ai += [float(COMP["Zone controller + vision accelerator option"]["peak_TOPS_int8"]) for z in zones if d.get("zone_compute", {}).get(z, "base") != "base"]
    left = (sum(ai) - max(ai)) * util if ai else 0.0
    if len(ai) < 2 or left + 1e-9 < red_need:
        errors.append(f"rule R: {len(ai)} computer(s) with AI capacity; after losing the largest, {left:.1f} usable TOPS remain (need {red_need:.0f})")
    # safety rules of the assignment
    def path(n): return d["nodes"][n]["attach"] if n in d["nodes"] else None
    front = [path(n) for n in ("C1", "R1", "L1")]
    if len(set(front)) == 1 and front[0] in zones: errors.append("rule S1: C1, R1 and L1 all depend on the same zone controller")
    if path("H1") == path("H2") and path("H1") != "CC": errors.append("rule S2: brake (H1) and steering (H2) on the same zone")
    missing = [n for n, r in NODES.items() if r["class"] not in ("compute", "zone_site") and n not in d["nodes"]]
    if missing: errors.append("nodes not connected: " + ", ".join(missing))
    print(f"{'link':38s} {'to':4s} {'type':12s} {'load Mbit/s':>12s} {'cap':>8s} {'length m':>9s}")
    for r in rows: print(f"{r[0][:38]:38s} {r[1]:4s} {r[2]:12s} {r[3]:12.2f} {r[4]:8.0f} {r[5]:9.2f}")
    if bus_orders:
        print("\nBus segments, cable order from one end to the other:")
        for b in bus_orders: print("  " + b)
    print("\nCost breakdown (USD, per vehicle):")
    for k, v in cost.items(): print(f"  {k:40s} {v:9.2f}")
    print(f"  {'TOTAL':40s} {sum(cost.values()):9.2f}")
    print(f"\nCable length total: {sum(r[5] for r in rows):.1f} m   cable mass: {mass_g/1000:.2f} kg   max electrical power (compute, zones, smart sensors): {power_W:.0f} W")
    print(f"TOPS needed: " + ", ".join(f"{k}={v:.0f}" for k, v in tops_need.items()) + f"   (usable = peak × {util})")
    print(f"Central compute: {have:.0f} peak TOPS, {have*util:.0f} usable, {tops_need['CC']:.0f} needed -> margin {have*util - tops_need['CC']:.0f} TOPS")
    print(f"Redundancy: {len(ai)} computer(s) with AI capacity; if the largest fails, {left:.1f} usable TOPS remain (need {red_need:.0f})")
    worst = max(rows, key=lambda r: r[3] / r[4] if r[4] else 0)
    print(f"Highest link utilisation: {100*worst[3]/worst[4]:.1f} % ({worst[0]}, {worst[2]}); limit {100*(1-head):.0f} %")
    print("\nCHECKS: " + ("all passed" if not errors else f"{len(errors)} problem(s)"))
    for e in errors: print("  - " + e)
    return 0 if not errors else 1

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "example_design_all_central.json"))
