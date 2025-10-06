<template>
  <div>
    <div class="card">
      <h2>System Status</h2>
      <div class="status-grid">
        <div class="status-item">
          <span class="status-indicator" :class="csiCollectorStatus"></span>
          <span>CSI Collector: {{ csiCollectorText }}</span>
        </div>
        <div class="status-item">
          <span class="status-indicator" :class="esp32Status"></span>
          <span>ESP32 Devices: {{ connectedDevices }}/{{ totalDevices }}</span>
          <span v-if="deviceModeInfo" class="device-mode-info">({{ deviceModeInfo }})</span>
        </div>
        <div class="status-item">
          <span class="status-indicator" :class="rosStatus"></span>
          <span>ROS2 Bridge: {{ rosEnabled ? rosText : 'Disabled' }}</span>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>Live CSI Data</h2>
      <div class="data-stats">
        <div class="stat">
          <h3>{{ packetsReceived }}</h3>
          <p>Packets Received</p>
        </div>
        <div class="stat">
          <h3>{{ dataRate }}</h3>
          <p>Data Rate (Hz)</p>
        </div>
        <div class="stat">
          <h3>{{ fileSize }}</h3>
          <p>Current File Size</p>
        </div>
      </div>
      
      <div class="chart-container">
        <canvas ref="chartCanvas"></canvas>
      </div>
    </div>

    <div class="card">
      <h2>Quick Actions</h2>
      <div class="action-buttons">
        <button class="btn btn-success" @click="startCollection" :disabled="isCollecting">
          Start Collection
        </button>
        <button class="btn btn-danger" @click="stopCollection" :disabled="!isCollecting">
          Stop Collection
        </button>
        <button class="btn" @click="refreshStatus">
          Refresh Status
        </button>
        <button class="btn" @click="downloadData">
          Download Data
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import axios from 'axios'
import { io } from 'socket.io-client'

export default {
  name: 'Dashboard',
  data() {
    return {
      csiCollectorStatus: 'status-connecting',
      csiCollectorText: 'Connecting...',
      esp32Status: 'status-disconnected',
      connectedDevices: 0,
      totalDevices: 0,
      rosStatus: 'status-disconnected',
      rosText: 'Disconnected',
      rosEnabled: false,
      packetsReceived: 0,
      dataRate: 0,
      fileSize: '0 KB',
      isCollecting: false,
      socket: null,
      deviceModeInfo: ''
    }
  },
  async mounted() {
    await this.initializeConnection()
    this.refreshStatus()
  },
  beforeUnmount() {
    if (this.socket) {
      this.socket.disconnect()
    }
  },
  methods: {
    async initializeConnection() {
      try {
        this.socket = io('/api')
        
        this.socket.on('connect', () => {
          this.csiCollectorStatus = 'status-connected'
          this.csiCollectorText = 'Connected'
        })
        
        this.socket.on('disconnect', () => {
          this.csiCollectorStatus = 'status-disconnected'
          this.csiCollectorText = 'Disconnected'
        })
        
        this.socket.on('csi_data', (data) => {
          this.packetsReceived = data.packets_received || 0
          this.dataRate = data.data_rate || 0
          this.fileSize = this.formatFileSize(data.file_size || 0)
        })
        
        this.socket.on('device_status', (data) => {
          this.connectedDevices = data.connected || 0
          this.totalDevices = data.total || 0
          this.esp32Status = this.connectedDevices > 0 ? 'status-connected' : 'status-disconnected'
          
          // Update device mode info
          if (data.devices && data.devices.length > 0) {
            const rxCount = data.devices.filter(d => d.mode === 'RX').length
            const txCount = data.devices.filter(d => d.mode === 'TX').length
            this.deviceModeInfo = `${rxCount} RX, ${txCount} TX`
          } else {
            this.deviceModeInfo = ''
          }
        })
        
      } catch (error) {
        console.error('Failed to initialize connection:', error)
        this.csiCollectorStatus = 'status-disconnected'
        this.csiCollectorText = 'Connection Failed'
      }
    },
    
    async refreshStatus() {
      try {
        const response = await axios.get('/api/status')
        const status = response.data
        
        this.isCollecting = status.collecting || false
        this.rosEnabled = status.ros_enabled || false
        
        if (this.rosEnabled) {
          this.rosStatus = status.ros_connected ? 'status-connected' : 'status-disconnected'
          this.rosText = status.ros_connected ? 'Connected' : 'Disconnected'
        }
        
      } catch (error) {
        console.error('Failed to refresh status:', error)
      }
    },
    
    async startCollection() {
      try {
        await axios.post('/api/start')
        this.isCollecting = true
      } catch (error) {
        console.error('Failed to start collection:', error)
        alert('Failed to start data collection')
      }
    },
    
    async stopCollection() {
      try {
        await axios.post('/api/stop')
        this.isCollecting = false
      } catch (error) {
        console.error('Failed to stop collection:', error)
        alert('Failed to stop data collection')
      }
    },
    
    async downloadData() {
      try {
        const response = await axios.get('/api/download', { responseType: 'blob' })
        const url = window.URL.createObjectURL(new Blob([response.data]))
        const link = document.createElement('a')
        link.href = url
        link.setAttribute('download', `csi_data_${Date.now()}.csv`)
        document.body.appendChild(link)
        link.click()
        link.remove()
      } catch (error) {
        console.error('Failed to download data:', error)
        alert('Failed to download data')
      }
    },
    
    formatFileSize(bytes) {
      if (bytes === 0) return '0 KB'
      const k = 1024
      const sizes = ['Bytes', 'KB', 'MB', 'GB']
      const i = Math.floor(Math.log(bytes) / Math.log(k))
      return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i]
    }
  }
}
</script>

<style scoped>
.status-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 1rem;
  margin-bottom: 1rem;
}

.status-item {
  display: flex;
  align-items: center;
  padding: 0.5rem;
  background: #f8f9fa;
  border-radius: 4px;
}

.data-stats {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 1rem;
  margin-bottom: 2rem;
}

.stat {
  text-align: center;
  padding: 1rem;
  background: #f8f9fa;
  border-radius: 4px;
}

.stat h3 {
  font-size: 2rem;
  margin-bottom: 0.5rem;
  color: #3498db;
}

.chart-container {
  height: 300px;
  margin-bottom: 1rem;
}

.device-mode-info {
  font-size: 0.875rem;
  color: #6c757d;
  margin-left: 0.5rem;
}

.action-buttons {
  display: flex;
  gap: 1rem;
  flex-wrap: wrap;
}
</style>