<template>
  <div>
    <div class="card">
      <h2>ESP32 Device Setup</h2>
      <p>Connect your ESP32 device via USB and configure it for CSI data collection.</p>
      
      <div class="setup-steps">
        <div class="step" :class="{ active: currentStep >= 1, completed: currentStep > 1 }">
          <div class="step-number">1</div>
          <div class="step-content">
            <h3>Detect Device</h3>
            <p>Connect ESP32 via USB and detect available ports</p>
            <button class="btn" @click="detectDevices" :disabled="isScanning">
              {{ isScanning ? 'Scanning...' : 'Scan for Devices' }}
            </button>
            
            <div v-if="availablePorts.length > 0" class="port-list">
              <h4>Available Ports:</h4>
              <div v-for="port in availablePorts" :key="port.device" class="port-item">
                <label>
                  <input v-model="selectedPort" :value="port.device" type="radio" />
                  {{ port.device }} - {{ port.description }}
                </label>
              </div>
            </div>
          </div>
        </div>

        <div class="step" :class="{ active: currentStep >= 2, completed: currentStep > 2 }">
          <div class="step-number">2</div>
          <div class="step-content">
            <h3>Flash ESP-CSI Firmware</h3>
            <p>Download and flash the ESP-CSI firmware to your device</p>
            
            <div class="firmware-options">
              <div class="firmware-option">
                <label>
                  <input v-model="firmwareType" value="stable" type="radio" />
                  Stable Release (Recommended)
                </label>
              </div>
              <div class="firmware-option">
                <label>
                  <input v-model="firmwareType" value="latest" type="radio" />
                  Latest Development
                </label>
              </div>
              <div class="firmware-option">
                <label>
                  <input v-model="firmwareType" value="custom" type="radio" />
                  Custom Firmware File
                </label>
                <input v-if="firmwareType === 'custom'" type="file" @change="handleFirmwareFile" accept=".bin" />
              </div>
            </div>
            
            <button class="btn btn-success" @click="flashFirmware" :disabled="!selectedPort || isFlashing">
              {{ isFlashing ? 'Flashing...' : 'Flash Firmware' }}
            </button>
          </div>
        </div>

        <div class="step" :class="{ active: currentStep >= 3, completed: currentStep > 3 }">
          <div class="step-number">3</div>
          <div class="step-content">
            <h3>Configure WiFi</h3>
            <p>Set up WiFi credentials and CSI parameters</p>
            
            <div class="config-form">
              <div class="form-group">
                <label>WiFi SSID:</label>
                <input v-model="wifiConfig.ssid" type="text" placeholder="Enter WiFi network name" />
              </div>
              
              <div class="form-group">
                <label>WiFi Password:</label>
                <input v-model="wifiConfig.password" type="password" placeholder="Enter WiFi password" />
              </div>
              
              <div class="form-group">
                <label>Device Mode:</label>
                <select v-model="wifiConfig.device_mode">
                  <option value="RX">RX (Receiver) - Passive CSI collection</option>
                  <option value="TX">TX (Transmitter) - Active packet transmission</option>
                </select>
              </div>
              
              <div class="form-group">
                <label>CSI Sampling Rate (Hz):</label>
                <input v-model.number="wifiConfig.csi_rate" type="number" min="1" max="100" />
              </div>
              
              <div class="form-group">
                <label>Channel:</label>
                <select v-model="wifiConfig.channel">
                  <option v-for="ch in availableChannels" :key="ch" :value="ch">{{ ch }}</option>
                </select>
              </div>
              
              <div class="form-group">
                <label>Bandwidth:</label>
                <select v-model="wifiConfig.bandwidth">
                  <option value="20">20 MHz</option>
                  <option value="40">40 MHz</option>
                </select>
              </div>
              
              <div v-if="wifiConfig.device_mode === 'TX'" class="tx-config-section">
                <h4>Transmitter Settings</h4>
                <div class="form-group">
                  <label>TX Power (dBm):</label>
                  <input v-model.number="wifiConfig.tx_power" type="number" min="0" max="20" />
                  <small>0-20 dBm (higher = stronger signal, more power)</small>
                </div>
                <div class="form-group">
                  <label>Beacon Interval (ms):</label>
                  <input v-model.number="wifiConfig.beacon_interval" type="number" min="50" max="1000" />
                  <small>Time between beacon transmissions</small>
                </div>
              </div>
              
              <div class="form-group">
                <label>
                  <input v-model="wifiConfig.enable_debug" type="checkbox" />
                  Enable Debug Mode
                </label>
              </div>
            </div>
            
            <button class="btn btn-success" @click="configureDevice" :disabled="!selectedPort || isConfiguring">
              {{ isConfiguring ? 'Configuring...' : 'Configure Device' }}
            </button>
          </div>
        </div>

        <div class="step" :class="{ active: currentStep >= 4, completed: currentStep > 4 }">
          <div class="step-number">4</div>
          <div class="step-content">
            <h3>Test Connection</h3>
            <p>Verify that the device is working correctly</p>
            
            <button class="btn" @click="testConnection" :disabled="!selectedPort || isTesting">
              {{ isTesting ? 'Testing...' : 'Test CSI Data Collection' }}
            </button>
            
            <div v-if="testResults" class="test-results">
              <h4>Test Results:</h4>
              <div class="result-item" :class="testResults.connection ? 'success' : 'error'">
                Connection: {{ testResults.connection ? 'Success' : 'Failed' }}
              </div>
              <div class="result-item" :class="testResults.data_flow ? 'success' : 'error'">
                Data Flow: {{ testResults.data_flow ? 'Success' : 'Failed' }}
              </div>
              <div v-if="testResults.packets_received" class="result-item success">
                Packets Received: {{ testResults.packets_received }}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Device Management</h2>
      
      <div class="device-list">
        <div class="device-list-header">
          <h3>Configured Devices ({{ configuredDevices.length }})</h3>
          <div class="device-stats">
            <span class="stat-badge stat-rx">{{ deviceStats.rx_count }} RX</span>
            <span class="stat-badge stat-tx">{{ deviceStats.tx_count }} TX</span>
            <span class="stat-badge stat-connected">{{ deviceStats.connected_count }} Connected</span>
          </div>
          <button class="btn" @click="loadConfiguredDevices">
            <i class="refresh-icon">🔄</i> Refresh
          </button>
        </div>
        
        <div v-if="configuredDevices.length === 0" class="no-devices">
          <p>No devices configured yet.</p>
          <p>Use the setup wizard above to configure your first ESP32 device.</p>
        </div>
        
        <div v-for="device in configuredDevices" :key="device.id" class="device-item">
          <div class="device-header">
            <div class="device-info">
              <h4>
                {{ device.name || device.id }}
                <span class="device-mode-badge" :class="device.device_mode.toLowerCase()">
                  {{ device.device_mode }}
                </span>
              </h4>
              <div class="device-meta">
                <span class="device-port">{{ device.port }}</span>
                <span class="device-location" v-if="device.location">📍 {{ device.location }}</span>
              </div>
            </div>
            
            <div class="device-status">
              <span class="status-indicator" :class="getStatusClass(device.status)"></span>
              <span class="status-text">{{ device.status }}</span>
            </div>
          </div>
          
          <div class="device-details">
            <div class="device-stats-grid">
              <div class="stat-item">
                <label>Signal:</label>
                <span>{{ device.signal_strength }} dBm</span>
              </div>
              <div class="stat-item">
                <label>Channel:</label>
                <span>{{ device.channel }}</span>
              </div>
              <div class="stat-item" v-if="device.device_mode === 'RX'">
                <label>Packets RX:</label>
                <span>{{ device.packets_received || 0 }}</span>
              </div>
              <div class="stat-item" v-if="device.device_mode === 'TX'">
                <label>Packets TX:</label>
                <span>{{ device.packets_transmitted || 0 }}</span>
              </div>
              <div class="stat-item" v-if="device.device_mode === 'TX'">
                <label>TX Power:</label>
                <span>{{ device.tx_power }} dBm</span>
              </div>
              <div class="stat-item">
                <label>Uptime:</label>
                <span>{{ device.uptime || 'Unknown' }}</span>
              </div>
            </div>
            
            <div class="device-actions">
              <button class="btn-small btn-info" @click="viewDeviceDetails(device)">
                📊 Details
              </button>
              <button class="btn-small btn-warning" @click="restartDevice(device)">
                🔄 Restart
              </button>
              <button class="btn-small btn-danger" @click="removeDevice(device)">
                🗑️ Remove
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Device Details Modal -->
    <div v-if="selectedDeviceDetails" class="modal-overlay" @click="closeDeviceDetails">
      <div class="modal-content device-details-modal" @click.stop>
        <div class="modal-header">
          <h3>{{ selectedDeviceDetails.name || selectedDeviceDetails.id }} Details</h3>
          <button class="modal-close" @click="closeDeviceDetails">×</button>
        </div>
        
        <div class="modal-body">
          <div class="detail-section">
            <h4>Basic Information</h4>
            <div class="detail-grid">
              <div class="detail-item">
                <label>Device ID:</label>
                <span>{{ selectedDeviceDetails.id }}</span>
              </div>
              <div class="detail-item">
                <label>Name:</label>
                <span>{{ selectedDeviceDetails.name }}</span>
              </div>
              <div class="detail-item">
                <label>Mode:</label>
                <span class="device-mode-badge" :class="selectedDeviceDetails.device_mode.toLowerCase()">
                  {{ selectedDeviceDetails.device_mode }}
                </span>
              </div>
              <div class="detail-item">
                <label>Status:</label>
                <span :class="getStatusClass(selectedDeviceDetails.status)">
                  {{ selectedDeviceDetails.status }}
                </span>
              </div>
              <div class="detail-item">
                <label>Location:</label>
                <span>{{ selectedDeviceDetails.location || 'Not specified' }}</span>
              </div>
              <div class="detail-item">
                <label>Port:</label>
                <span>{{ selectedDeviceDetails.port }}</span>
              </div>
            </div>
          </div>

          <div class="detail-section">
            <h4>Hardware Information</h4>
            <div class="detail-grid">
              <div class="detail-item">
                <label>Hardware:</label>
                <span>{{ selectedDeviceDetails.hardware_revision || 'Unknown' }}</span>
              </div>
              <div class="detail-item">
                <label>Firmware:</label>
                <span>{{ selectedDeviceDetails.firmware_version || 'Unknown' }}</span>
              </div>
              <div class="detail-item">
                <label>MAC Address:</label>
                <span class="monospace">{{ selectedDeviceDetails.mac_address || 'Unknown' }}</span>
              </div>
              <div class="detail-item">
                <label>Setup Time:</label>
                <span>{{ formatTimestamp(selectedDeviceDetails.setup_time) }}</span>
              </div>
              <div class="detail-item">
                <label>Last Seen:</label>
                <span>{{ formatTimestamp(selectedDeviceDetails.last_seen) }}</span>
              </div>
            </div>
          </div>

          <div class="detail-section" v-if="selectedDeviceDetails.configuration">
            <h4>Configuration</h4>
            <div class="detail-grid">
              <div class="detail-item" v-for="(value, key) in selectedDeviceDetails.configuration" :key="key">
                <label>{{ formatConfigKey(key) }}:</label>
                <span>{{ formatConfigValue(key, value) }}</span>
              </div>
            </div>
          </div>

          <div class="detail-section" v-if="selectedDeviceDetails.statistics">
            <h4>Statistics</h4>
            <div class="detail-grid">
              <div class="detail-item" v-for="(value, key) in selectedDeviceDetails.statistics" :key="key">
                <label>{{ formatStatKey(key) }}:</label>
                <span>{{ formatStatValue(key, value) }}</span>
              </div>
            </div>
          </div>

          <div class="detail-section" v-if="selectedDeviceDetails.health">
            <h4>Health Status</h4>
            <div class="detail-grid">
              <div class="detail-item" v-for="(value, key) in selectedDeviceDetails.health" :key="key" v-if="value !== null">
                <label>{{ formatHealthKey(key) }}:</label>
                <span>{{ formatHealthValue(key, value) }}</span>
              </div>
            </div>
          </div>
        </div>

        <div class="modal-footer">
          <button class="btn" @click="closeDeviceDetails">Close</button>
          <button class="btn btn-warning" @click="restartDeviceFromDetails">Restart Device</button>
        </div>
      </div>
    </div>

    <!-- Progress Modal -->
    <div v-if="showProgressModal" class="modal-overlay">
      <div class="modal-content">
        <h3>{{ progressTitle }}</h3>
        <div class="progress-bar">
          <div class="progress-fill" :style="{ width: progressPercent + '%' }"></div>
        </div>
        <p>{{ progressMessage }}</p>
        
        <div v-if="progressLogs.length > 0" class="progress-logs">
          <div v-for="(log, index) in progressLogs" :key="index" class="log-entry">
            {{ log }}
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import axios from 'axios'

export default {
  name: 'ESP32Setup',
  data() {
    return {
      currentStep: 1,
      isScanning: false,
      isFlashing: false,
      isConfiguring: false,
      isTesting: false,
      availablePorts: [],
      selectedPort: '',
      firmwareType: 'stable',
      customFirmwareFile: null,
      wifiConfig: {
        ssid: '',
        password: '',
        device_mode: 'RX',
        csi_rate: 10,
        channel: 6,
        bandwidth: 20,
        tx_power: 15,
        beacon_interval: 100,
        enable_debug: false
      },
      availableChannels: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
      testResults: null,
      configuredDevices: [],
      deviceStats: {
        total_count: 0,
        rx_count: 0,
        tx_count: 0,
        connected_count: 0
      },
      selectedDeviceDetails: null,
      showProgressModal: false,
      progressTitle: '',
      progressMessage: '',
      progressPercent: 0,
      progressLogs: []
    }
  },
  async mounted() {
    await this.loadConfiguredDevices()
  },
  methods: {
    async detectDevices() {
      this.isScanning = true
      try {
        const response = await axios.get('/api/esp32/scan')
        this.availablePorts = response.data.ports || []
        if (this.availablePorts.length > 0) {
          this.currentStep = Math.max(this.currentStep, 2)
        }
      } catch (error) {
        console.error('Failed to scan for devices:', error)
        alert('Failed to scan for devices')
      } finally {
        this.isScanning = false
      }
    },
    
    async flashFirmware() {
      if (!this.selectedPort) return
      
      this.isFlashing = true
      this.showProgressModal = true
      this.progressTitle = 'Flashing Firmware'
      this.progressMessage = 'Preparing to flash...'
      this.progressPercent = 0
      this.progressLogs = []
      
      try {
        const formData = new FormData()
        formData.append('port', this.selectedPort)
        formData.append('firmware_type', this.firmwareType)
        
        if (this.firmwareType === 'custom' && this.customFirmwareFile) {
          formData.append('firmware_file', this.customFirmwareFile)
        }
        
        const response = await axios.post('/api/esp32/flash', formData, {
          headers: {
            'Content-Type': 'multipart/form-data'
          },
          onUploadProgress: (progressEvent) => {
            this.progressPercent = Math.round((progressEvent.loaded * 100) / progressEvent.total)
          }
        })
        
        if (response.data.success) {
          this.currentStep = Math.max(this.currentStep, 3)
          this.progressMessage = 'Firmware flashed successfully!'
          this.progressPercent = 100
        } else {
          throw new Error(response.data.error || 'Flash failed')
        }
        
      } catch (error) {
        console.error('Failed to flash firmware:', error)
        this.progressMessage = 'Flash failed: ' + (error.response?.data?.error || error.message)
      } finally {
        this.isFlashing = false
        setTimeout(() => {
          this.showProgressModal = false
        }, 2000)
      }
    },
    
    async configureDevice() {
      if (!this.selectedPort) return
      
      this.isConfiguring = true
      try {
        const response = await axios.post('/api/esp32/configure', {
          port: this.selectedPort,
          wifi_ssid: this.wifiConfig.ssid,
          wifi_password: this.wifiConfig.password,
          device_mode: this.wifiConfig.device_mode,
          csi_rate: this.wifiConfig.csi_rate,
          channel: this.wifiConfig.channel,
          bandwidth: this.wifiConfig.bandwidth,
          tx_power: this.wifiConfig.tx_power,
          beacon_interval: this.wifiConfig.beacon_interval,
          enable_debug: this.wifiConfig.enable_debug
        })
        
        if (response.data.success) {
          this.currentStep = Math.max(this.currentStep, 4)
          alert('Device configured successfully!')
          await this.loadConfiguredDevices()
        } else {
          throw new Error(response.data.error || 'Configuration failed')
        }
        
      } catch (error) {
        console.error('Failed to configure device:', error)
        alert('Failed to configure device: ' + (error.response?.data?.error || error.message))
      } finally {
        this.isConfiguring = false
      }
    },
    
    async testConnection() {
      if (!this.selectedPort) return
      
      this.isTesting = true
      this.testResults = null
      
      try {
        const response = await axios.post('/api/esp32/test', {
          port: this.selectedPort
        })
        
        this.testResults = response.data
        
      } catch (error) {
        console.error('Failed to test connection:', error)
        this.testResults = {
          connection: false,
          data_flow: false,
          error: error.response?.data?.error || error.message
        }
      } finally {
        this.isTesting = false
      }
    },
    
    async loadConfiguredDevices() {
      try {
        const response = await axios.get('/api/esp32/devices')
        this.configuredDevices = response.data.devices || []
        this.deviceStats = response.data.stats || {
          total_count: 0,
          rx_count: 0,
          tx_count: 0,
          connected_count: 0
        }
      } catch (error) {
        console.error('Failed to load configured devices:', error)
      }
    },
    
    async removeDevice(device) {
      if (!confirm(`Are you sure you want to remove device "${device.name || device.id}"? This action cannot be undone.`)) {
        return
      }

      try {
        await axios.delete(`/api/esp32/device/${device.id}`)
        await this.loadConfiguredDevices() // Refresh device list
        alert(`Device "${device.name || device.id}" has been removed`)
      } catch (error) {
        console.error('Failed to remove device:', error)
        alert('Failed to remove device')
      }
    },
    
    async viewDeviceDetails(device) {
      try {
        const response = await axios.get(`/api/esp32/device/${device.id}`)
        this.selectedDeviceDetails = response.data
      } catch (error) {
        console.error('Failed to load device details:', error)
        alert('Failed to load device details')
      }
    },

    closeDeviceDetails() {
      this.selectedDeviceDetails = null
    },

    async restartDevice(device) {
      if (!confirm(`Are you sure you want to restart device "${device.name || device.id}"?`)) {
        return
      }

      try {
        await axios.post(`/api/esp32/device/${device.id}/restart`)
        alert(`Device "${device.name || device.id}" restart command sent`)
        await this.loadConfiguredDevices() // Refresh device list
      } catch (error) {
        console.error('Failed to restart device:', error)
        alert('Failed to restart device')
      }
    },

    async restartDeviceFromDetails() {
      if (this.selectedDeviceDetails) {
        await this.restartDevice(this.selectedDeviceDetails)
        this.closeDeviceDetails()
      }
    },

    getStatusClass(status) {
      switch (status) {
        case 'connected': return 'status-connected'
        case 'disconnected': return 'status-disconnected'
        case 'error': return 'status-error'
        case 'unknown': return 'status-unknown'
        default: return 'status-unknown'
      }
    },

    formatTimestamp(timestamp) {
      if (!timestamp) return 'Unknown'
      try {
        return new Date(timestamp).toLocaleString()
      } catch (e) {
        return 'Invalid date'
      }
    },

    formatConfigKey(key) {
      return key.replace(/_/g, ' ')
        .replace(/\b\w/g, l => l.toUpperCase())
    },

    formatConfigValue(key, value) {
      if (value === null || value === undefined) return 'Not set'
      if (typeof value === 'boolean') return value ? 'Enabled' : 'Disabled'
      if (key.includes('power') && typeof value === 'number') return `${value} dBm`
      if (key.includes('interval') && typeof value === 'number') return `${value} ms`
      if (key.includes('rate') && typeof value === 'number') return `${value} Hz`
      return String(value)
    },

    formatStatKey(key) {
      return key.replace(/_/g, ' ')
        .replace(/\b\w/g, l => l.toUpperCase())
    },

    formatStatValue(key, value) {
      if (value === null || value === undefined) return '0'
      if (key.includes('bytes')) {
        const bytes = parseInt(value)
        if (bytes > 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
        if (bytes > 1024) return `${(bytes / 1024).toFixed(1)} KB`
        return `${bytes} B`
      }
      if (key.includes('rate') && typeof value === 'number') return `${value.toFixed(2)}/s`
      return String(value)
    },

    formatHealthKey(key) {
      return key.replace(/_/g, ' ')
        .replace(/\b\w/g, l => l.toUpperCase())
    },

    formatHealthValue(key, value) {
      if (value === null || value === undefined) return 'Unknown'
      if (key.includes('temperature') && typeof value === 'number') return `${value}°C`
      if (key.includes('memory') && typeof value === 'number') return `${value}%`
      if (key.includes('cpu') && typeof value === 'number') return `${value}%`
      return String(value)
    },
    
    handleFirmwareFile(event) {
      this.customFirmwareFile = event.target.files[0]
    }
  }
}
</script>

<style scoped>
.setup-steps {
  margin: 2rem 0;
}

.step {
  display: flex;
  margin-bottom: 2rem;
  padding: 1.5rem;
  border: 2px solid #e9ecef;
  border-radius: 8px;
  transition: all 0.3s ease;
}

.step.active {
  border-color: #3498db;
  background: #f8f9fa;
}

.step.completed {
  border-color: #27ae60;
  background: #d4edda;
}

.step-number {
  flex-shrink: 0;
  width: 40px;
  height: 40px;
  background: #e9ecef;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: bold;
  margin-right: 1.5rem;
}

.step.active .step-number {
  background: #3498db;
  color: white;
}

.step.completed .step-number {
  background: #27ae60;
  color: white;
}

.step-content {
  flex: 1;
}

.step-content h3 {
  margin-bottom: 0.5rem;
  color: #2c3e50;
}

.step-content p {
  margin-bottom: 1rem;
  color: #6c757d;
}

.port-list {
  margin-top: 1rem;
  padding: 1rem;
  background: white;
  border-radius: 4px;
}

.port-item {
  margin-bottom: 0.5rem;
}

.port-item label {
  display: flex;
  align-items: center;
  cursor: pointer;
}

.port-item input {
  margin-right: 0.5rem;
}

.firmware-options {
  margin: 1rem 0;
}

.firmware-option {
  margin-bottom: 0.5rem;
}

.firmware-option label {
  display: flex;
  align-items: center;
  cursor: pointer;
}

.firmware-option input[type="radio"] {
  margin-right: 0.5rem;
}

.firmware-option input[type="file"] {
  margin-left: 1rem;
  margin-top: 0.5rem;
}

.config-form {
  margin: 1rem 0;
}

.config-form .form-group {
  margin-bottom: 1rem;
}

.config-form label {
  display: block;
  margin-bottom: 0.5rem;
  font-weight: 500;
}

.config-form input,
.config-form select {
  width: 100%;
  max-width: 300px;
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.tx-config-section {
  margin-top: 1.5rem;
  padding: 1rem;
  background: #e8f4fd;
  border-radius: 4px;
  border-left: 4px solid #007bff;
}

.tx-config-section h4 {
  margin-bottom: 1rem;
  color: #007bff;
  font-size: 1.1rem;
}

.config-form small {
  display: block;
  margin-top: 0.25rem;
  color: #6c757d;
  font-size: 0.875rem;
}

.test-results {
  margin-top: 1rem;
  padding: 1rem;
  background: white;
  border-radius: 4px;
}

.result-item {
  padding: 0.5rem;
  margin-bottom: 0.5rem;
  border-radius: 4px;
}

.result-item.success {
  background: #d4edda;
  color: #155724;
}

.result-item.error {
  background: #f8d7da;
  color: #721c24;
}

.device-list {
  background: white;
  border-radius: 8px;
  padding: 16px;
  margin-top: 1rem;
}

.device-list-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  border-bottom: 1px solid #e5e7eb;
  padding-bottom: 12px;
}

.device-list-header h3 {
  margin: 0;
  color: #374151;
}

.device-stats {
  display: flex;
  gap: 8px;
}

.stat-badge {
  display: inline-block;
  padding: 4px 8px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: 600;
  color: white;
}

.stat-badge.stat-rx {
  background: #10b981;
}

.stat-badge.stat-tx {
  background: #f59e0b;
}

.stat-badge.stat-connected {
  background: #3b82f6;
}

.no-devices {
  text-align: center;
  padding: 40px 20px;
  color: #6b7280;
  background: #f9fafb;
  border-radius: 8px;
  border: 2px dashed #d1d5db;
}

.device-item {
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  margin-bottom: 12px;
  padding: 16px;
  background: #fafafa;
  transition: all 0.2s ease;
}

.device-item:hover {
  box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
}

.device-header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 12px;
}

.device-info h4 {
  margin: 0 0 4px 0;
  color: #374151;
  display: flex;
  align-items: center;
  gap: 8px;
}

.device-mode-badge {
  display: inline-block;
  padding: 2px 6px;
  border-radius: 6px;
  font-size: 10px;
  font-weight: 600;
  text-transform: uppercase;
  color: white;
}

.device-mode-badge.rx {
  background: #10b981;
}

.device-mode-badge.tx {
  background: #f59e0b;
}

.device-meta {
  display: flex;
  gap: 12px;
  font-size: 13px;
  color: #6b7280;
}

.device-port {
  font-family: monospace;
  background: #f3f4f6;
  padding: 2px 4px;
  border-radius: 3px;
}

.device-status {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 13px;
}

.status-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}

.status-indicator.status-connected {
  background: #10b981;
}

.status-indicator.status-disconnected {
  background: #6b7280;
}

.status-indicator.status-error {
  background: #ef4444;
}

.status-indicator.status-unknown {
  background: #f59e0b;
}

.device-details {
  border-top: 1px solid #e5e7eb;
  padding-top: 12px;
}

.device-stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
  gap: 8px;
  margin-bottom: 12px;
}

.stat-item {
  font-size: 12px;
}

.stat-item label {
  display: block;
  color: #6b7280;
  font-weight: 500;
  margin-bottom: 2px;
}

.stat-item span {
  color: #374151;
  font-weight: 600;
}

.device-actions {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
}

.btn-small {
  padding: 6px 12px;
  font-size: 12px;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s ease;
  font-weight: 500;
}

.btn-small.btn-info {
  background: #3b82f6;
  color: white;
}

.btn-small.btn-info:hover {
  background: #2563eb;
}

.btn-small.btn-warning {
  background: #f59e0b;
  color: white;
}

.btn-small.btn-warning:hover {
  background: #d97706;
}

.btn-small.btn-danger {
  background: #ef4444;
  color: white;
}

.btn-small.btn-danger:hover {
  background: #dc2626;
}

/* Modal Styles */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal-content {
  background: white;
  border-radius: 8px;
  box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
  max-height: 90vh;
  overflow-y: auto;
}

.device-details-modal {
  width: 90%;
  max-width: 800px;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px;
  border-bottom: 1px solid #e5e7eb;
}

.modal-header h3 {
  margin: 0;
  color: #374151;
}

.modal-close {
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  color: #6b7280;
  padding: 4px;
  line-height: 1;
}

.modal-close:hover {
  color: #374151;
}

.modal-body {
  padding: 20px;
  max-height: 60vh;
  overflow-y: auto;
}

.modal-footer {
  padding: 16px 20px;
  border-top: 1px solid #e5e7eb;
  display: flex;
  justify-content: flex-end;
  gap: 12px;
}

.detail-section {
  margin-bottom: 24px;
}

.detail-section h4 {
  margin: 0 0 12px 0;
  color: #374151;
  font-size: 16px;
  font-weight: 600;
  border-bottom: 1px solid #e5e7eb;
  padding-bottom: 8px;
}

.detail-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 12px;
}

.detail-item {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.detail-item label {
  font-size: 12px;
  color: #6b7280;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.detail-item span {
  color: #374151;
  font-weight: 500;
}

.detail-item .monospace {
  font-family: monospace;
  background: #f3f4f6;
  padding: 4px 6px;
  border-radius: 4px;
  font-size: 13px;
}

.refresh-icon {
  display: inline-block;
  transition: transform 0.3s ease;
}

.btn:hover .refresh-icon {
  transform: rotate(180deg);
}

/* Status text colors */
.status-connected {
  color: #10b981;
  font-weight: 600;
}

.status-disconnected {
  color: #6b7280;
  font-weight: 600;
}

.status-error {
  color: #ef4444;
  font-weight: 600;
}

.status-unknown {
  color: #f59e0b;
  font-weight: 600;
}

.progress-bar {
  width: 100%;
  height: 20px;
  background: #e9ecef;
  border-radius: 10px;
  overflow: hidden;
  margin: 1rem 0;
}

.progress-fill {
  height: 100%;
  background: #3498db;
  transition: width 0.3s ease;
}

.progress-logs {
  max-height: 200px;
  overflow-y: auto;
  background: #f8f9fa;
  padding: 1rem;
  border-radius: 4px;
  margin-top: 1rem;
}

.log-entry {
  font-family: monospace;
  font-size: 0.875rem;
  margin-bottom: 0.25rem;
}
</style>