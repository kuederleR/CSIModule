<template>
  <div>
    <div class="card">
      <h2>System Configuration</h2>
      <form @submit.prevent="saveConfiguration">
        <div class="form-section">
          <h3>Serial Communication</h3>
          <div class="form-group">
            <label>Serial Port:</label>
            <input v-model="config.serial.port" type="text" placeholder="/dev/ttyUSB0" />
          </div>
          <div class="form-group">
            <label>Baud Rate:</label>
            <select v-model="config.serial.baudrate">
              <option value="115200">115200</option>
              <option value="921600">921600</option>
              <option value="460800">460800</option>
            </select>
          </div>
        </div>

        <div class="form-section">
          <h3>ROS2 Integration</h3>
          <div class="form-group">
            <label>
              <input v-model="config.ros2.enabled" type="checkbox" />
              Enable ROS2 Communication
            </label>
          </div>
          <div v-if="config.ros2.enabled" class="form-group">
            <label>ROS Domain ID:</label>
            <input v-model.number="config.ros2.domain_id" type="number" min="0" max="101" />
          </div>
          <div v-if="config.ros2.enabled" class="form-group">
            <label>Topic Name:</label>
            <input v-model="config.ros2.topic" type="text" placeholder="/csi_data" />
          </div>
        </div>

        <div class="form-section">
          <h3>Data Collection</h3>
          <div class="form-group">
            <label>Output Directory:</label>
            <input v-model="config.data.output_dir" type="text" placeholder="/app/data" />
          </div>
          <div class="form-group">
            <label>File Format:</label>
            <select v-model="config.data.format">
              <option value="csv">CSV</option>
              <option value="json">JSON</option>
              <option value="binary">Binary</option>
            </select>
          </div>
          <div class="form-group">
            <label>Max File Size (MB):</label>
            <input v-model.number="config.data.max_file_size" type="number" min="1" />
          </div>
        </div>

        <div class="form-section">
          <h3>ESP32 Configuration</h3>
          <div class="form-group">
            <label>WiFi SSID:</label>
            <input v-model="config.esp32.wifi_ssid" type="text" />
          </div>
          <div class="form-group">
            <label>WiFi Password:</label>
            <input v-model="config.esp32.wifi_password" type="password" />
          </div>
          <div class="form-group">
            <label>Device Mode:</label>
            <select v-model="config.esp32.device_mode">
              <option value="RX">RX (Receiver) - Passive CSI collection</option>
              <option value="TX">TX (Transmitter) - Active packet transmission</option>
            </select>
          </div>
          <div class="form-group">
            <label>CSI Rate (Hz):</label>
            <input v-model.number="config.esp32.csi_rate" type="number" min="1" max="100" />
          </div>
          <div class="form-group">
            <label>WiFi Channel:</label>
            <select v-model.number="config.esp32.channel">
              <option v-for="ch in availableChannels" :key="ch" :value="ch">{{ ch }}</option>
            </select>
          </div>
          <div class="form-group">
            <label>Bandwidth (MHz):</label>
            <select v-model.number="config.esp32.bandwidth">
              <option value="20">20 MHz</option>
              <option value="40">40 MHz</option>
            </select>
          </div>
          <div v-if="config.esp32.device_mode === 'TX'" class="tx-specific-config">
            <h4>Transmitter Settings</h4>
            <div class="form-group">
              <label>TX Power (dBm):</label>
              <input v-model.number="config.esp32.tx_power" type="number" min="0" max="20" />
              <small>Higher values = stronger signal, more power consumption</small>
            </div>
            <div class="form-group">
              <label>Beacon Interval (ms):</label>
              <input v-model.number="config.esp32.beacon_interval" type="number" min="50" max="1000" />
              <small>Time between beacon transmissions</small>
            </div>
            <div class="form-group">
              <label>Probe Interval (ms):</label>
              <input v-model.number="config.esp32.probe_interval" type="number" min="100" max="5000" />
              <small>Time between probe request transmissions</small>
            </div>
          </div>
          <div class="form-group">
            <label>Filter MAC Address:</label>
            <input v-model="config.esp32.filter_mac" type="text" placeholder="AA:BB:CC:DD:EE:FF (optional)" />
            <small>Leave empty to capture all devices</small>
          </div>
          <div class="form-group">
            <label>
              <input v-model="config.esp32.enable_debug" type="checkbox" />
              Enable Debug Output
            </label>
          </div>
        </div>

        <div class="form-actions">
          <button type="submit" class="btn btn-success">Save Configuration</button>
          <button type="button" class="btn" @click="loadConfiguration">Reload</button>
          <button type="button" class="btn" @click="resetToDefaults">Reset to Defaults</button>
        </div>
      </form>
    </div>
  </div>
</template>

<script>
import axios from 'axios'

export default {
  name: 'Configuration',
  data() {
    return {
      config: {
        serial: {
          port: '/dev/ttyUSB0',
          baudrate: 115200
        },
        ros2: {
          enabled: false,
          domain_id: 0,
          topic: '/csi_data'
        },
        data: {
          output_dir: '/app/data',
          format: 'csv',
          max_file_size: 100
        },
        esp32: {
          wifi_ssid: '',
          wifi_password: '',
          csi_rate: 10,
          channel: 6,
          bandwidth: 20,
          device_mode: 'RX',
          tx_power: 20,
          beacon_interval: 100,
          probe_interval: 1000,
          filter_mac: '',
          enable_debug: false
        }
      },
      availableChannels: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
    }
  },
  async mounted() {
    await this.loadConfiguration()
  },
  methods: {
    async loadConfiguration() {
      try {
        const response = await axios.get('/api/config')
        this.config = { ...this.config, ...response.data }
      } catch (error) {
        console.error('Failed to load configuration:', error)
        alert('Failed to load configuration')
      }
    },
    
    async saveConfiguration() {
      try {
        await axios.post('/api/config', this.config)
        alert('Configuration saved successfully!')
      } catch (error) {
        console.error('Failed to save configuration:', error)
        alert('Failed to save configuration')
      }
    },
    
    resetToDefaults() {
      this.config = {
        serial: {
          port: '/dev/ttyUSB0',
          baudrate: 115200
        },
        ros2: {
          enabled: false,
          domain_id: 0,
          topic: '/csi_data'
        },
        data: {
          output_dir: '/app/data',
          format: 'csv',
          max_file_size: 100
        },
        esp32: {
          wifi_ssid: '',
          wifi_password: '',
          csi_rate: 10,
          channel: 6,
          bandwidth: 20,
          device_mode: 'RX',
          tx_power: 20,
          beacon_interval: 100,
          probe_interval: 1000,
          filter_mac: '',
          enable_debug: false
        }
      }
    }
  }
}
</script>

<style scoped>
.form-section {
  margin-bottom: 2rem;
  padding-bottom: 1.5rem;
  border-bottom: 1px solid #eee;
}

.form-section:last-of-type {
  border-bottom: none;
}

.form-section h3 {
  margin-bottom: 1rem;
  color: #2c3e50;
}

.form-group {
  margin-bottom: 1rem;
}

.form-group label {
  display: block;
  margin-bottom: 0.5rem;
  font-weight: 500;
}

.form-group input,
.form-group select {
  width: 100%;
  max-width: 400px;
  padding: 0.75rem;
  border: 1px solid #ddd;
  border-radius: 4px;
  font-size: 1rem;
}

.form-group input[type="checkbox"] {
  width: auto;
  margin-right: 0.5rem;
}

.tx-specific-config {
  margin-top: 1.5rem;
  padding: 1rem;
  background: #f8f9fa;
  border-radius: 4px;
  border-left: 4px solid #007bff;
}

.tx-specific-config h4 {
  margin-bottom: 1rem;
  color: #007bff;
  font-size: 1.1rem;
}

.form-group small {
  display: block;
  margin-top: 0.25rem;
  color: #6c757d;
  font-size: 0.875rem;
}

.form-actions {
  display: flex;
  gap: 1rem;
  margin-top: 2rem;
}
</style>