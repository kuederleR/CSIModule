#!/usr/bin/env python3
"""
Example script for analyzing WiFi CSI data collected by the CSI Collector system.
This script demonstrates various analysis techniques for CSI data.
"""

import argparse
import json
import logging
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from scipy import signal
from scipy.fft import fft, fftfreq
import seaborn as sns

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class CSIAnalyzer:
    """Analyzer for WiFi CSI data"""
    
    def __init__(self, data_file: str):
        """Initialize analyzer with data file"""
        self.data_file = Path(data_file)
        self.data = None
        self.load_data()
    
    def load_data(self):
        """Load CSI data from file"""
        logger.info(f"Loading data from {self.data_file}")
        
        if self.data_file.suffix == '.csv':
            self.data = pd.read_csv(self.data_file)
            
            # Parse JSON columns
            if 'amplitude' in self.data.columns:
                self.data['amplitude_data'] = self.data['amplitude'].apply(
                    lambda x: json.loads(x) if isinstance(x, str) else []
                )
            
            if 'phase' in self.data.columns:
                self.data['phase_data'] = self.data['phase'].apply(
                    lambda x: json.loads(x) if isinstance(x, str) else []
                )
        
        elif self.data_file.suffix == '.json':
            with open(self.data_file, 'r') as f:
                json_data = [json.loads(line) for line in f]
            self.data = pd.DataFrame(json_data)
        
        else:
            raise ValueError(f"Unsupported file format: {self.data_file.suffix}")
        
        # Convert timestamp to datetime
        if 'timestamp' in self.data.columns:
            self.data['datetime'] = pd.to_datetime(self.data['timestamp'], unit='s')
        
        logger.info(f"Loaded {len(self.data)} records")
    
    def basic_statistics(self):
        """Calculate basic statistics"""
        logger.info("Calculating basic statistics...")
        
        stats = {
            'total_packets': len(self.data),
            'duration_seconds': 0,
            'avg_rssi': 0,
            'rssi_std': 0,
            'channels_used': [],
            'avg_subcarriers': 0
        }
        
        if 'timestamp' in self.data.columns and len(self.data) > 1:
            stats['duration_seconds'] = self.data['timestamp'].max() - self.data['timestamp'].min()
            stats['data_rate_hz'] = len(self.data) / stats['duration_seconds']
        
        if 'rssi' in self.data.columns:
            stats['avg_rssi'] = self.data['rssi'].mean()
            stats['rssi_std'] = self.data['rssi'].std()
            stats['rssi_min'] = self.data['rssi'].min()
            stats['rssi_max'] = self.data['rssi'].max()
        
        if 'channel' in self.data.columns:
            stats['channels_used'] = sorted(self.data['channel'].unique().tolist())
        
        if 'amplitude_data' in self.data.columns:
            lengths = self.data['amplitude_data'].apply(len)
            stats['avg_subcarriers'] = lengths.mean()
            stats['subcarrier_range'] = [lengths.min(), lengths.max()]
        
        return stats
    
    def plot_rssi_timeline(self, save_path: str = None):
        """Plot RSSI over time"""
        logger.info("Plotting RSSI timeline...")
        
        plt.figure(figsize=(12, 6))
        
        if 'datetime' in self.data.columns and 'rssi' in self.data.columns:
            plt.plot(self.data['datetime'], self.data['rssi'], alpha=0.7, linewidth=0.5)
            plt.xlabel('Time')
            plt.ylabel('RSSI (dBm)')
            plt.title('RSSI Over Time')
            plt.grid(True, alpha=0.3)
            
            # Add moving average
            window_size = min(50, len(self.data) // 10)
            if window_size > 1:
                rolling_mean = self.data['rssi'].rolling(window=window_size).mean()
                plt.plot(self.data['datetime'], rolling_mean, 'r-', 
                        linewidth=2, label=f'Moving Average ({window_size} samples)')
                plt.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"RSSI timeline saved to {save_path}")
        else:
            plt.show()
    
    def plot_amplitude_heatmap(self, save_path: str = None, max_samples: int = 1000):
        """Plot CSI amplitude as heatmap"""
        logger.info("Plotting CSI amplitude heatmap...")
        
        if 'amplitude_data' not in self.data.columns:
            logger.warning("No amplitude data found")
            return
        
        # Sample data if too large
        if len(self.data) > max_samples:
            sample_data = self.data.sample(n=max_samples).sort_index()
        else:
            sample_data = self.data
        
        # Extract amplitude matrix
        amplitudes = []
        for _, row in sample_data.iterrows():
            amp_data = row['amplitude_data']
            if isinstance(amp_data, list) and len(amp_data) > 0:
                amplitudes.append(amp_data)
        
        if not amplitudes:
            logger.warning("No valid amplitude data found")
            return
        
        # Pad to same length
        max_length = max(len(amp) for amp in amplitudes)
        amplitude_matrix = np.array([
            amp + [0] * (max_length - len(amp)) for amp in amplitudes
        ])
        
        plt.figure(figsize=(12, 8))
        
        # Create heatmap
        im = plt.imshow(amplitude_matrix.T, aspect='auto', cmap='viridis',
                       extent=[0, len(amplitudes), 0, max_length])
        
        plt.colorbar(im, label='Amplitude')
        plt.xlabel('Time Sample')
        plt.ylabel('Subcarrier Index')
        plt.title('CSI Amplitude Heatmap')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Amplitude heatmap saved to {save_path}")
        else:
            plt.show()
    
    def analyze_frequency_domain(self, subcarrier_idx: int = 0, save_path: str = None):
        """Analyze frequency domain characteristics"""
        logger.info(f"Analyzing frequency domain for subcarrier {subcarrier_idx}...")
        
        if 'amplitude_data' not in self.data.columns:
            logger.warning("No amplitude data found")
            return
        
        # Extract time series for specific subcarrier
        amplitudes = []
        timestamps = []
        
        for _, row in self.data.iterrows():
            amp_data = row['amplitude_data']
            if isinstance(amp_data, list) and len(amp_data) > subcarrier_idx:
                amplitudes.append(amp_data[subcarrier_idx])
                timestamps.append(row['timestamp'])
        
        if len(amplitudes) < 10:
            logger.warning("Insufficient data for frequency analysis")
            return
        
        amplitudes = np.array(amplitudes)
        timestamps = np.array(timestamps)
        
        # Calculate sampling rate
        dt = np.mean(np.diff(timestamps))
        fs = 1.0 / dt if dt > 0 else 1.0
        
        # Perform FFT
        fft_data = fft(amplitudes)
        freqs = fftfreq(len(amplitudes), dt)
        
        # Take positive frequencies only
        positive_freq_idx = freqs > 0
        freqs_pos = freqs[positive_freq_idx]
        fft_pos = np.abs(fft_data[positive_freq_idx])
        
        # Plot frequency domain
        plt.figure(figsize=(12, 6))
        
        plt.subplot(2, 1, 1)
        plt.plot(timestamps, amplitudes)
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title(f'Time Domain - Subcarrier {subcarrier_idx}')
        plt.grid(True, alpha=0.3)
        
        plt.subplot(2, 1, 2)
        plt.plot(freqs_pos, fft_pos)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude')
        plt.title(f'Frequency Domain - Subcarrier {subcarrier_idx}')
        plt.grid(True, alpha=0.3)
        plt.xlim(0, min(fs/2, 1.0))  # Limit to reasonable frequency range
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Frequency analysis saved to {save_path}")
        else:
            plt.show()
        
        return {
            'sampling_rate': fs,
            'dominant_frequency': freqs_pos[np.argmax(fft_pos)] if len(fft_pos) > 0 else 0,
            'frequency_range': [freqs_pos.min(), freqs_pos.max()] if len(freqs_pos) > 0 else [0, 0]
        }
    
    def detect_activity(self, threshold_factor: float = 2.0):
        """Detect activity based on RSSI variance"""
        logger.info("Detecting activity based on RSSI variance...")
        
        if 'rssi' not in self.data.columns:
            logger.warning("No RSSI data found")
            return []
        
        # Calculate rolling statistics
        window_size = min(50, len(self.data) // 10)
        if window_size < 5:
            logger.warning("Insufficient data for activity detection")
            return []
        
        rolling_mean = self.data['rssi'].rolling(window=window_size, center=True).mean()
        rolling_std = self.data['rssi'].rolling(window=window_size, center=True).std()
        
        # Detect anomalies
        threshold = rolling_std.mean() * threshold_factor
        anomaly_mask = np.abs(self.data['rssi'] - rolling_mean) > threshold
        
        # Find activity periods
        activity_periods = []
        in_activity = False
        start_idx = None
        
        for idx, is_anomaly in enumerate(anomaly_mask):
            if is_anomaly and not in_activity:
                start_idx = idx
                in_activity = True
            elif not is_anomaly and in_activity:
                activity_periods.append((start_idx, idx))
                in_activity = False
        
        # Close final period if needed
        if in_activity:
            activity_periods.append((start_idx, len(self.data) - 1))
        
        logger.info(f"Detected {len(activity_periods)} activity periods")
        
        return activity_periods
    
    def correlation_analysis(self, max_subcarriers: int = 10):
        """Analyze correlation between subcarriers"""
        logger.info("Performing correlation analysis...")
        
        if 'amplitude_data' not in self.data.columns:
            logger.warning("No amplitude data found")
            return None
        
        # Extract amplitude matrix (limited subcarriers)
        amplitudes = []
        for _, row in self.data.iterrows():
            amp_data = row['amplitude_data']
            if isinstance(amp_data, list) and len(amp_data) >= max_subcarriers:
                amplitudes.append(amp_data[:max_subcarriers])
        
        if len(amplitudes) < 10:
            logger.warning("Insufficient data for correlation analysis")
            return None
        
        amplitude_matrix = np.array(amplitudes)
        
        # Calculate correlation matrix
        correlation_matrix = np.corrcoef(amplitude_matrix.T)
        
        # Plot correlation heatmap
        plt.figure(figsize=(10, 8))
        sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0,
                   square=True, linewidths=0.5)
        plt.title('Subcarrier Correlation Matrix')
        plt.xlabel('Subcarrier Index')
        plt.ylabel('Subcarrier Index')
        
        plt.tight_layout()
        plt.show()
        
        return correlation_matrix
    
    def generate_report(self, output_dir: str = None):
        """Generate comprehensive analysis report"""
        logger.info("Generating analysis report...")
        
        if output_dir:
            output_path = Path(output_dir)
            output_path.mkdir(exist_ok=True)
        else:
            output_path = self.data_file.parent / f"{self.data_file.stem}_analysis"
            output_path.mkdir(exist_ok=True)
        
        # Basic statistics
        stats = self.basic_statistics()
        
        # Generate plots
        self.plot_rssi_timeline(str(output_path / "rssi_timeline.png"))
        self.plot_amplitude_heatmap(str(output_path / "amplitude_heatmap.png"))
        
        if 'amplitude_data' in self.data.columns:
            freq_analysis = self.analyze_frequency_domain(0, str(output_path / "frequency_analysis.png"))
            stats.update(freq_analysis or {})
        
        # Activity detection
        activity_periods = self.detect_activity()
        
        # Generate report text
        report_text = f"""
# CSI Data Analysis Report

## Data Overview
- **File**: {self.data_file.name}
- **Total Packets**: {stats['total_packets']:,}
- **Duration**: {stats['duration_seconds']:.2f} seconds
- **Data Rate**: {stats.get('data_rate_hz', 0):.2f} Hz

## Signal Characteristics
- **Average RSSI**: {stats['avg_rssi']:.2f} dBm
- **RSSI Standard Deviation**: {stats['rssi_std']:.2f} dB
- **RSSI Range**: [{stats.get('rssi_min', 0):.1f}, {stats.get('rssi_max', 0):.1f}] dBm
- **Channels Used**: {stats['channels_used']}
- **Average Subcarriers**: {stats['avg_subcarriers']:.1f}

## Activity Analysis
- **Activity Periods Detected**: {len(activity_periods)}
- **Sampling Rate**: {stats.get('sampling_rate', 0):.2f} Hz
- **Dominant Frequency**: {stats.get('dominant_frequency', 0):.3f} Hz

## Generated Files
- rssi_timeline.png: RSSI variation over time
- amplitude_heatmap.png: CSI amplitude visualization
- frequency_analysis.png: Frequency domain analysis

## Analysis Notes
This analysis provides basic insights into the CSI data characteristics.
For advanced analysis, consider:
- Machine learning-based activity recognition
- Multi-antenna array processing
- Environmental factor correlation
- Long-term pattern analysis
"""
        
        # Save report
        report_file = output_path / "analysis_report.md"
        with open(report_file, 'w') as f:
            f.write(report_text)
        
        logger.info(f"Analysis report generated in {output_path}")
        
        return stats


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='CSI Data Analysis Tool')
    parser.add_argument('data_file', help='Path to CSI data file')
    parser.add_argument('--output', '-o', help='Output directory for analysis results')
    parser.add_argument('--report', action='store_true', help='Generate full analysis report')
    parser.add_argument('--stats', action='store_true', help='Show basic statistics only')
    parser.add_argument('--plot-rssi', action='store_true', help='Plot RSSI timeline')
    parser.add_argument('--plot-heatmap', action='store_true', help='Plot amplitude heatmap')
    parser.add_argument('--frequency-analysis', type=int, metavar='SUBCARRIER',
                       help='Perform frequency analysis for specified subcarrier')
    parser.add_argument('--activity-detection', action='store_true',
                       help='Detect activity periods')
    parser.add_argument('--correlation', action='store_true',
                       help='Analyze subcarrier correlations')
    
    args = parser.parse_args()
    
    try:
        # Initialize analyzer
        analyzer = CSIAnalyzer(args.data_file)
        
        if args.report:
            # Generate full report
            analyzer.generate_report(args.output)
        else:
            # Individual analysis options
            if args.stats:
                stats = analyzer.basic_statistics()
                print("\nBasic Statistics:")
                for key, value in stats.items():
                    print(f"  {key}: {value}")
            
            if args.plot_rssi:
                analyzer.plot_rssi_timeline()
            
            if args.plot_heatmap:
                analyzer.plot_amplitude_heatmap()
            
            if args.frequency_analysis is not None:
                analyzer.analyze_frequency_domain(args.frequency_analysis)
            
            if args.activity_detection:
                periods = analyzer.detect_activity()
                print(f"\nDetected {len(periods)} activity periods:")
                for i, (start, end) in enumerate(periods):
                    print(f"  Period {i+1}: samples {start}-{end}")
            
            if args.correlation:
                analyzer.correlation_analysis()
    
    except Exception as e:
        logger.error(f"Analysis failed: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())