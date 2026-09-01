import csv
import math
from pathlib import Path
import re
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix

from gps_navigator.geo_utils import gps_to_local_xy, valid_nav_sat_fix


CSV_FIELDS = [
    'timestamp', 'elapsed_s', 'latitude', 'longitude', 'altitude',
    'local_east_m', 'local_north_m', 'displacement_from_first_m',
    'displacement_from_final_recommended_m', 'fix_status',
    'covariance_x', 'covariance_y', 'covariance_z',
]


def percentile(values, percentage):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * percentage
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def linear_slope(times, values):
    if len(times) < 2:
        return 0.0
    mean_t = statistics.fmean(times)
    mean_v = statistics.fmean(values)
    denominator = sum((value - mean_t) ** 2 for value in times)
    if denominator <= 0.0:
        return 0.0
    return sum(
        (stamp - mean_t) * (value - mean_v)
        for stamp, value in zip(times, values)
    ) / denominator


def coordinate_from_local(samples, east, north):
    reference = samples[0]
    latitude = reference['latitude'] + math.degrees(
        north / 6371000.0
    )
    mean_latitude = math.radians(
        0.5 * (reference['latitude'] + latitude)
    )
    longitude = reference['longitude'] + math.degrees(
        east / (6371000.0 * math.cos(mean_latitude))
    )
    return latitude, longitude


def subset_statistics(samples, indices):
    east = [samples[index]['east_m'] for index in indices]
    north = [samples[index]['north_m'] for index in indices]
    times = [samples[index]['elapsed_s'] for index in indices]
    median_east = statistics.median(east)
    median_north = statistics.median(north)
    radii = [
        math.hypot(x - median_east, y - median_north)
        for x, y in zip(east, north)
    ]
    east_slope = linear_slope(times, east)
    north_slope = linear_slope(times, north)
    return {
        'mean_east_m': statistics.fmean(east),
        'mean_north_m': statistics.fmean(north),
        'median_east_m': median_east,
        'median_north_m': median_north,
        'radius_95_m': percentile(radii, 0.95),
        'rms_spread_m': math.sqrt(statistics.fmean(r * r for r in radii)),
        'east_drift_m_min': east_slope * 60.0,
        'north_drift_m_min': north_slope * 60.0,
        'drift_m_min': math.hypot(east_slope, north_slope) * 60.0,
    }


def analyze_samples(samples):
    """Return robust waypoint statistics for timestamped local ENU samples."""
    if len(samples) < 3:
        raise ValueError('at least three valid GPS samples are required')
    all_indices = list(range(len(samples)))
    full = subset_statistics(samples, all_indices)
    windows = []
    for number in range(3):
        start = number * len(samples) // 3
        end = (number + 1) * len(samples) // 3
        indices = list(range(start, end))
        windows.append(subset_statistics(samples, indices))

    duration = samples[-1]['elapsed_s'] - samples[0]['elapsed_s']
    trend_displacement = full['drift_m_min'] * duration / 60.0
    drift_threshold = max(2.0, 2.0 * full['radius_95_m'])
    monotonic_drift = (
        duration >= 60.0
        and full['drift_m_min'] >= 0.2
        and trend_displacement >= drift_threshold
    )

    best_window_index = min(
        range(3), key=lambda index: windows[index]['radius_95_m']
    )
    stable_after_convergence = (
        best_window_index > 0
        and windows[0]['radius_95_m']
        > 1.5 * max(windows[best_window_index]['radius_95_m'], 0.05)
        and windows[best_window_index]['drift_m_min'] < 0.2
    )
    if stable_after_convergence:
        selected = windows[best_window_index]
        method = ('middle' if best_window_index == 1 else 'late') + '_median'
    else:
        selected = full
        method = 'full_session_median'

    recommended_east = selected['median_east_m']
    recommended_north = selected['median_north_m']
    recommended_lat, recommended_lon = coordinate_from_local(
        samples, recommended_east, recommended_north
    )
    mean_lat, mean_lon = coordinate_from_local(
        samples, full['mean_east_m'], full['mean_north_m']
    )
    median_lat, median_lon = coordinate_from_local(
        samples, full['median_east_m'], full['median_north_m']
    )
    distances_from_recommended = [
        math.hypot(
            sample['east_m'] - recommended_east,
            sample['north_m'] - recommended_north,
        )
        for sample in samples
    ]
    if monotonic_drift:
        confidence = 'POOR'
        warning = (
            'monotonic GNSS drift detected; waypoint absolute position may '
            'be biased by several meters'
        )
    elif selected['radius_95_m'] <= 2.0:
        confidence = 'GOOD'
        warning = ''
    else:
        confidence = 'FAIR'
        warning = 'GNSS scatter exceeds the 2 m GOOD-confidence radius'

    return {
        'duration_s': duration,
        'sample_count': len(samples),
        'first_coordinate': (samples[0]['latitude'], samples[0]['longitude']),
        'last_coordinate': (samples[-1]['latitude'], samples[-1]['longitude']),
        'mean_coordinate': (mean_lat, mean_lon),
        'median_coordinate': (median_lat, median_lon),
        'recommended_coordinate': (recommended_lat, recommended_lon),
        'recommended_local_m': (recommended_east, recommended_north),
        'recommended_method': method,
        'confidence': confidence,
        'warning': warning,
        'start_end_drift_m': math.hypot(
            samples[-1]['east_m'] - samples[0]['east_m'],
            samples[-1]['north_m'] - samples[0]['north_m'],
        ),
        'maximum_displacement_m': max(distances_from_recommended),
        'rms_horizontal_spread_m': full['rms_spread_m'],
        'radius_95_m': full['radius_95_m'],
        'east_range_m': (min(s['east_m'] for s in samples),
                         max(s['east_m'] for s in samples)),
        'north_range_m': (min(s['north_m'] for s in samples),
                          max(s['north_m'] for s in samples)),
        'east_drift_m_min': full['east_drift_m_min'],
        'north_drift_m_min': full['north_drift_m_min'],
        'drift_m_min': full['drift_m_min'],
        'trend_displacement_m': trend_displacement,
        'monotonic_drift': monotonic_drift,
        'windows': windows,
    }


def covariance_values(msg):
    if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None, None, None
    values = (msg.position_covariance[0], msg.position_covariance[4],
              msg.position_covariance[8])
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        return None, None, None
    return values


class GpsWaypointSampler(Node):

    def __init__(self):
        super().__init__('gps_waypoint_sampler')
        self.declare_parameter('waypoint_id', 'WAYPOINT')
        self.declare_parameter('duration_s', 600.0)
        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter(
            'position_topic', '/mavros/global_position/global'
        )
        self.declare_parameter(
            'raw_fix_topic', '/mavros/global_position/raw/fix'
        )
        self.duration_s = float(self.get_parameter('duration_s').value)
        if self.duration_s <= 0.0:
            raise ValueError('duration_s must be positive')
        waypoint = str(self.get_parameter('waypoint_id').value).strip()
        self.waypoint_id = waypoint or 'WAYPOINT'
        safe_id = re.sub(r'[^A-Za-z0-9_.-]+', '_', self.waypoint_id)
        output_dir = Path(str(self.get_parameter('output_dir').value))
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        base = output_dir / f'gps_waypoint_{safe_id}_{stamp}'
        self.csv_path = base.with_suffix('.csv')
        self.png_path = base.with_suffix('.png')
        self.samples = []
        self.first_receive_ns = None
        self.latest_raw = None
        self.latest_raw_receive_ns = None
        self.finished = False
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix, self.get_parameter('position_topic').value,
            self.position_callback, qos,
        )
        raw_topic = str(self.get_parameter('raw_fix_topic').value).strip()
        if raw_topic:
            self.create_subscription(
                NavSatFix, raw_topic, self.raw_callback, qos
            )
        self.create_timer(0.2, self.check_duration)
        self.get_logger().info(
            f'GPS waypoint sampler ready: waypoint={self.waypoint_id}, '
            f'duration={self.duration_s:.1f}s, CSV={self.csv_path}'
        )

    def raw_callback(self, msg):
        self.latest_raw = msg
        self.latest_raw_receive_ns = self.get_clock().now().nanoseconds

    def position_callback(self, msg):
        if self.finished or not valid_nav_sat_fix(msg):
            return
        receive_ns = self.get_clock().now().nanoseconds
        if self.first_receive_ns is None:
            self.first_receive_ns = receive_ns
            self.reference = (msg.latitude, msg.longitude)
        east, north = gps_to_local_xy(
            msg.latitude, msg.longitude, *self.reference
        )
        diagnostic = msg
        if (
            self.latest_raw is not None
            and receive_ns - self.latest_raw_receive_ns <= 1_000_000_000
        ):
            diagnostic = self.latest_raw
        covariance = covariance_values(diagnostic)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.samples.append({
            'timestamp': stamp if stamp > 0.0 else receive_ns / 1e9,
            'elapsed_s': (receive_ns - self.first_receive_ns) / 1e9,
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'east_m': east,
            'north_m': north,
            'fix_status': diagnostic.status.status,
            'covariance_x': covariance[0],
            'covariance_y': covariance[1],
            'covariance_z': covariance[2],
        })

    def check_duration(self):
        if self.first_receive_ns is None or self.finished:
            return
        elapsed = (
            self.get_clock().now().nanoseconds - self.first_receive_ns
        ) / 1e9
        if elapsed >= self.duration_s:
            self.finish()

    def write_csv(self, analysis):
        east, north = analysis['recommended_local_m']
        with self.csv_path.open('w', newline='', encoding='utf-8') as output:
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for sample in self.samples:
                first_distance = math.hypot(
                    sample['east_m'], sample['north_m']
                )
                recommended_distance = math.hypot(
                    sample['east_m'] - east, sample['north_m'] - north
                )
                row = {
                    **sample,
                    'local_east_m': sample['east_m'],
                    'local_north_m': sample['north_m'],
                    'displacement_from_first_m': first_distance,
                    'displacement_from_final_recommended_m': (
                        recommended_distance
                    ),
                }
                writer.writerow({
                    key: '' if row.get(key) is None else row.get(key)
                    for key in CSV_FIELDS
                })

    def write_plot(self, analysis):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warning(
                'matplotlib unavailable; PNG not created'
            )
            return False
        times = [sample['elapsed_s'] for sample in self.samples]
        east = [sample['east_m'] for sample in self.samples]
        north = [sample['north_m'] for sample in self.samples]
        recommended = analysis['recommended_local_m']
        displacement = [
            math.hypot(x - recommended[0], y - recommended[1])
            for x, y in zip(east, north)
        ]
        figure, axes = plt.subplots(2, 2, figsize=(11, 8))
        axes[0, 0].plot(east, north, '-o', markersize=2)
        axes[0, 0].scatter(*recommended, marker='*', s=100,
                           label='recommended')
        axes[0, 0].set(xlabel='East (m)', ylabel='North (m)',
                       title='Stationary GPS trajectory')
        axes[0, 0].axis('equal')
        axes[0, 0].legend()
        axes[0, 1].plot(times, east)
        axes[0, 1].set(xlabel='Elapsed (s)', ylabel='East (m)')
        axes[1, 0].plot(times, north)
        axes[1, 0].set(xlabel='Elapsed (s)', ylabel='North (m)')
        axes[1, 1].plot(times, displacement)
        axes[1, 1].set(xlabel='Elapsed (s)', ylabel='Displacement (m)')
        for axis in axes.flat:
            axis.grid(True)
        figure.tight_layout()
        figure.savefig(self.png_path, dpi=160)
        plt.close(figure)
        return True

    def print_result(self, analysis, png_created):
        def coordinate(value):
            return f'{value[0]:.8f}, {value[1]:.8f}'
        east_range = analysis['east_range_m']
        north_range = analysis['north_range_m']
        lines = [
            '[GPS WAYPOINT SAMPLER]',
            f'waypoint: {self.waypoint_id}',
            f'duration: {analysis["duration_s"]:.1f} s',
            f'valid samples: {analysis["sample_count"]}',
            f'first coordinate: {coordinate(analysis["first_coordinate"])}',
            f'last coordinate: {coordinate(analysis["last_coordinate"])}',
            f'mean coordinate: {coordinate(analysis["mean_coordinate"])}',
            f'median coordinate: {coordinate(analysis["median_coordinate"])}',
            'recommended coordinate: '
            f'{coordinate(analysis["recommended_coordinate"])}',
            f'recommended method: {analysis["recommended_method"]}',
            f'confidence: {analysis["confidence"]}',
            f'total start->end drift: {analysis["start_end_drift_m"]:.3f} m',
            'maximum displacement: '
            f'{analysis["maximum_displacement_m"]:.3f} m',
            'RMS horizontal spread: '
            f'{analysis["rms_horizontal_spread_m"]:.3f} m',
            f'95% horizontal radius: {analysis["radius_95_m"]:.3f} m',
            f'east/west range: {east_range[0]:.3f} .. {east_range[1]:.3f} m',
            'north/south range: '
            f'{north_range[0]:.3f} .. {north_range[1]:.3f} m',
            f'east drift rate: {analysis["east_drift_m_min"]:+.3f} m/min',
            f'north drift rate: {analysis["north_drift_m_min"]:+.3f} m/min',
            'total horizontal drift rate: '
            f'{analysis["drift_m_min"]:.3f} m/min',
        ]
        if analysis['warning']:
            lines.append(f'WARNING: {analysis["warning"]}')
        latitude, longitude = analysis['recommended_coordinate']
        lines.extend([
            f'CSV: {self.csv_path}',
            f'PNG: {self.png_path if png_created else "not created"}',
            '',
            f'{self.waypoint_id}:',
            f'  latitude: {latitude:.8f}',
            f'  longitude: {longitude:.8f}',
        ])
        self.get_logger().info('\n'.join(lines))

    def finish(self):
        if self.finished:
            return
        self.finished = True
        if len(self.samples) < 3:
            self.get_logger().error(
                'Sampling ended with fewer than three valid GPS samples'
            )
            return
        analysis = analyze_samples(self.samples)
        self.write_csv(analysis)
        png_created = self.write_plot(analysis)
        self.print_result(analysis, png_created)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GpsWaypointSampler()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted; analyzing collected samples')
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
