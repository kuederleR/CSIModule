<template>
  <div>
    <div class="card">
      <h2>Data Viewer</h2>
      <div class="viewer-controls">
        <div class="control-group">
          <label>Data File:</label>
          <select v-model="selectedFile" @change="loadFile">
            <option value="">Select a file...</option>
            <option v-for="file in dataFiles" :key="file" :value="file">{{ file }}</option>
          </select>
        </div>
        
        <div class="control-group">
          <button class="btn" @click="refreshFiles">Refresh Files</button>
          <button class="btn" @click="exportData" :disabled="!selectedFile">Export</button>
        </div>
      </div>
    </div>

    <div v-if="currentData" class="card">
      <h2>Data Analysis</h2>
      <div class="data-info">
        <div class="info-item">
          <strong>Total Records:</strong> {{ currentData.length }}
        </div>
        <div class="info-item">
          <strong>Date Range:</strong> {{ dateRange }}
        </div>
        <div class="info-item">
          <strong>Devices:</strong> {{ uniqueDevices }}
        </div>
      </div>

      <div class="chart-container">
        <canvas ref="amplitudeChart"></canvas>
      </div>
      
      <div class="chart-container">
        <canvas ref="phaseChart"></canvas>
      </div>
    </div>

    <div v-if="currentData" class="card">
      <h2>Raw Data Table</h2>
      <div class="table-controls">
        <input v-model="searchTerm" type="text" placeholder="Search..." class="search-input" />
        <div class="pagination">
          <button @click="prevPage" :disabled="currentPage === 1">Previous</button>
          <span>Page {{ currentPage }} of {{ totalPages }}</span>
          <button @click="nextPage" :disabled="currentPage === totalPages">Next</button>
        </div>
      </div>
      
      <div class="table-container">
        <table class="data-table">
          <thead>
            <tr>
              <th>Timestamp</th>
              <th>Device ID</th>
              <th>RSSI</th>
              <th>Channel</th>
              <th>Data Length</th>
              <th>Actions</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="row in paginatedData" :key="row.id">
              <td>{{ formatTimestamp(row.timestamp) }}</td>
              <td>{{ row.device_id }}</td>
              <td>{{ row.rssi }}</td>
              <td>{{ row.channel }}</td>
              <td>{{ row.data_length }}</td>
              <td>
                <button class="btn-small" @click="viewDetails(row)">Details</button>
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>

    <!-- Details Modal -->
    <div v-if="selectedRecord" class="modal-overlay" @click="closeDetails">
      <div class="modal-content" @click.stop>
        <h3>CSI Data Details</h3>
        <div class="detail-grid">
          <div><strong>Timestamp:</strong> {{ formatTimestamp(selectedRecord.timestamp) }}</div>
          <div><strong>Device ID:</strong> {{ selectedRecord.device_id }}</div>
          <div><strong>RSSI:</strong> {{ selectedRecord.rssi }} dBm</div>
          <div><strong>Channel:</strong> {{ selectedRecord.channel }}</div>
          <div><strong>Bandwidth:</strong> {{ selectedRecord.bandwidth }} MHz</div>
          <div><strong>Data Length:</strong> {{ selectedRecord.data_length }}</div>
        </div>
        
        <div class="csi-data-visualization">
          <h4>CSI Amplitude</h4>
          <canvas ref="detailChart"></canvas>
        </div>
        
        <button class="btn" @click="closeDetails">Close</button>
      </div>
    </div>
  </div>
</template>

<script>
import axios from 'axios'

export default {
  name: 'DataViewer',
  data() {
    return {
      dataFiles: [],
      selectedFile: '',
      currentData: null,
      selectedRecord: null,
      searchTerm: '',
      currentPage: 1,
      itemsPerPage: 20
    }
  },
  computed: {
    filteredData() {
      if (!this.currentData || !this.searchTerm) {
        return this.currentData || []
      }
      return this.currentData.filter(row => 
        Object.values(row).some(value => 
          String(value).toLowerCase().includes(this.searchTerm.toLowerCase())
        )
      )
    },
    
    paginatedData() {
      const start = (this.currentPage - 1) * this.itemsPerPage
      const end = start + this.itemsPerPage
      return this.filteredData.slice(start, end)
    },
    
    totalPages() {
      return Math.ceil(this.filteredData.length / this.itemsPerPage)
    },
    
    dateRange() {
      if (!this.currentData || this.currentData.length === 0) return 'N/A'
      
      const timestamps = this.currentData.map(row => new Date(row.timestamp))
      const min = new Date(Math.min(...timestamps))
      const max = new Date(Math.max(...timestamps))
      
      return `${min.toLocaleDateString()} - ${max.toLocaleDateString()}`
    },
    
    uniqueDevices() {
      if (!this.currentData) return 0
      return new Set(this.currentData.map(row => row.device_id)).size
    }
  },
  async mounted() {
    await this.refreshFiles()
  },
  methods: {
    async refreshFiles() {
      try {
        const response = await axios.get('/api/data/files')
        this.dataFiles = response.data.files || []
      } catch (error) {
        console.error('Failed to load data files:', error)
        alert('Failed to load data files')
      }
    },
    
    async loadFile() {
      if (!this.selectedFile) {
        this.currentData = null
        return
      }
      
      try {
        const response = await axios.get(`/api/data/file/${this.selectedFile}`)
        this.currentData = response.data.data || []
        this.currentPage = 1
      } catch (error) {
        console.error('Failed to load data file:', error)
        alert('Failed to load data file')
      }
    },
    
    async exportData() {
      if (!this.selectedFile) return
      
      try {
        const response = await axios.get(`/api/data/export/${this.selectedFile}`, {
          responseType: 'blob'
        })
        
        const url = window.URL.createObjectURL(new Blob([response.data]))
        const link = document.createElement('a')
        link.href = url
        link.setAttribute('download', this.selectedFile)
        document.body.appendChild(link)
        link.click()
        link.remove()
      } catch (error) {
        console.error('Failed to export data:', error)
        alert('Failed to export data')
      }
    },
    
    viewDetails(record) {
      this.selectedRecord = record
    },
    
    closeDetails() {
      this.selectedRecord = null
    },
    
    prevPage() {
      if (this.currentPage > 1) {
        this.currentPage--
      }
    },
    
    nextPage() {
      if (this.currentPage < this.totalPages) {
        this.currentPage++
      }
    },
    
    formatTimestamp(timestamp) {
      return new Date(timestamp).toLocaleString()
    }
  }
}
</script>

<style scoped>
.viewer-controls {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
  flex-wrap: wrap;
  gap: 1rem;
}

.control-group {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.control-group label {
  font-weight: 500;
}

.control-group select {
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
}

.data-info {
  display: flex;
  gap: 2rem;
  margin-bottom: 1.5rem;
  flex-wrap: wrap;
}

.info-item {
  padding: 0.75rem;
  background: #f8f9fa;
  border-radius: 4px;
}

.chart-container {
  height: 300px;
  margin-bottom: 2rem;
}

.table-controls {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
  flex-wrap: wrap;
  gap: 1rem;
}

.search-input {
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
  width: 300px;
}

.pagination {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.table-container {
  overflow-x: auto;
}

.data-table {
  width: 100%;
  border-collapse: collapse;
}

.data-table th,
.data-table td {
  padding: 0.75rem;
  text-align: left;
  border-bottom: 1px solid #ddd;
}

.data-table th {
  background: #f8f9fa;
  font-weight: 600;
}

.btn-small {
  background: #6c757d;
  color: white;
  border: none;
  padding: 0.25rem 0.5rem;
  border-radius: 3px;
  cursor: pointer;
  font-size: 0.875rem;
}

.btn-small:hover {
  background: #5a6268;
}

.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 1000;
}

.modal-content {
  background: white;
  padding: 2rem;
  border-radius: 8px;
  max-width: 800px;
  max-height: 90vh;
  overflow-y: auto;
  width: 90%;
}

.detail-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
  margin-bottom: 1.5rem;
}

.detail-grid > div {
  padding: 0.5rem;
  background: #f8f9fa;
  border-radius: 4px;
}

.csi-data-visualization {
  margin: 1.5rem 0;
}

.csi-data-visualization h4 {
  margin-bottom: 1rem;
}

.csi-data-visualization canvas {
  width: 100%;
  height: 200px;
}
</style>