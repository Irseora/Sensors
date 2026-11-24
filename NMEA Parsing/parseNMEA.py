import math
import matplotlib.pyplot as plt

def to_degrees(raw, direction):
    """
    NMEA encodes angular coords:
        - Latitude: DDMM.MMMM = 2 digits degrees + rest minutes
        - Longitude: DDDMM.MMMM = 3 digits degrees (-180..180)
    => Need to convert to decimal degrees:
        DecDeg = Degrees + (Minutes / 60)
    South and West are encoded as negative values by ISO standards.
    """
    if not raw or not direction:
        return None

    dir_len = 3 if direction in ('E', 'W') else 2
    deg = float(raw[:dir_len])
    mins = float(raw[dir_len:])
    val = round(deg + mins / 60.0, 6)

    return -val if direction in ('S', 'W') else val


def haversine(lat1, lon1, lat2, lon2):
    """
    Haversine formula yields more accurate metric step displacements than Euclidean distance.
    Earth modeled as a sphere of mean volumetric radius R = 6.371.000 meters.
    """
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2)**2

    return 2.0 * R * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def parse_file(filename):
    epochs = []
    curr = {}
    last_time = None

    with open(filename, 'r', errors='ignore') as f:
        for line in f:
            line = line.strip().rstrip('\\')
            
            p = line[1:].split('*')[0].split(',')
            tag = p[0]

            #GGA: Timestamp, Coords, HDOP (Precision), Altitude
            if tag.endswith('GGA'):
                t = p[1]

                # Commit prev epoch on timestamp change
                if last_time and t != last_time and curr:
                    epochs.append(curr)
                    curr = {}
                
                last_time = t
                curr['time'] = t
                curr['fix'] = int(p[6]) if p[6] else 0
                curr['lat'] = to_degrees(p[2], p[3])
                curr['lon'] = to_degrees(p[4], p[5])
                curr['hdop'] = float(p[8]) if p[8] else None
                curr['alt'] = float(p[9]) if p[9] else None

            #VTG: Ground speed in km/h
            elif tag.endswith('VTG'):
                curr['speed'] = float(p[7]) if len(p) > 7 and p[7] else 0.0

            #RMC: Date of Fix, Signal Validity Status
            elif tag.endswith('RMC'):
                curr['valid'] = (p[2] == 'A')   # A = Active, V = Void
                if len(p) > 9 and p[9]:
                    curr['date'] = p[9]

    if curr:
        epochs.append(curr)
        
    return epochs


def check_questions(epochs, file_label):
    valid_pts = []
    speeds = []
    hdops = []
    dates = set()
    total_dist = 0.0

    prev_pt = None
    for ep in epochs:
        # Valid GGA fix and active RMC status
        is_valid = ep.get('fix', 0) > 0 and ep.get('valid', True)
        if is_valid and ep.get('lat') is not None and ep.get('lon') is not None:
            pt = (ep['lat'], ep['lon'])
            valid_pts.append(pt)

            if prev_pt:
                step = haversine(prev_pt[0], prev_pt[1], pt[0], pt[1])
                # Sensor noise filter
                if step > 0.4:
                    total_dist += step

            prev_pt = pt

        if 'speed' in ep:
            speeds.append(ep['speed'])
        if ep.get('hdop'):
            hdops.append(ep['hdop'])
        if ep.get('date'):
            dates.add(ep['date'])

    max_speed = max(speeds) if speeds else 0.0
    avg_speed = sum(speeds) / len(speeds) if speeds else 0.0
    typical_hdop = sorted(hdops)[len(hdops)//2] if hdops else 0.0
    est_accuracy = round(typical_hdop * 2.5, 2)

    # Calculate net straight-line displacement
    net_displacement = haversine(valid_pts[0][0], valid_pts[0][1], valid_pts[-1][0], valid_pts[-1][1]) if len(valid_pts) > 1 else 0.0

    is_moving = net_displacement > 25.0 and max_speed > 2.0
    status = "Moving" if is_moving else "Stationary"

    # Location calculation
    if valid_pts:
        mean_lat = sum(p[0] for p in valid_pts) / len(valid_pts)
        mean_lon = sum(p[1] for p in valid_pts) / len(valid_pts)
        loc_str = f"{mean_lat:.5f}° N, {mean_lon:.5f}° E (Budapest)"
    else:
        loc_str = "No valid coordinates acquired."

    print(f"{file_label} Results")
    print(f"Q1 Movement: {status}")
    print(f"\t- Total Path: {total_dist:.2f} m | Net Displacement: {net_displacement:.2f} m")
    print(f"\t- Top Speed: {max_speed:.2f} km/h | Average Speed: {avg_speed:.2f} km/h")
    print(f"\t- Location: {loc_str}")
    print(f"Q2 Precision & Accuracy:")
    print(f"\t- Typical HDOP: {typical_hdop}")
    print(f"\t- Estimated Accuracy: ~{est_accuracy} meters")
    print(f"Q3 Additional Info:")
    print(f"\t- Date Recorded: {', '.join(dates) if dates else 'N/A'}")
    print(f"\t- Valid GPS Fix Epochs: {len(valid_pts)} / {len(epochs)}")

    # Visualize path
    plt.figure()
    plt.plot([p[1] for p in valid_pts], [p[0] for p in valid_pts], 'm.-')
    plt.gca().set_aspect(1.0 / math.cos(math.radians(valid_pts[0][0])))
    plt.title(f"{file_label}: {status}")
    out_name = f"{file_label.lower().replace(' ', '_')}_plot.png"
    plt.savefig(out_name)
    plt.close()
    print(f"\t- Plot saved to: {out_name}")

file1 = parse_file('nmea-test1-1.txt')
file2 = parse_file('nmea-test2-1.txt')

check_questions(file1, "File 1")
print("\n\n\n")
check_questions(file2, "File 2")