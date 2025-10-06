import { createApp } from 'vue'
import { createRouter, createWebHistory } from 'vue-router'
import App from './App.vue'
import Dashboard from './components/Dashboard.vue'
import Configuration from './components/Configuration.vue'
import DataViewer from './components/DataViewer.vue'
import ESP32Setup from './components/ESP32Setup.vue'

const routes = [
  { path: '/', component: Dashboard },
  { path: '/config', component: Configuration },
  { path: '/data', component: DataViewer },
  { path: '/setup', component: ESP32Setup }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

const app = createApp(App)
app.use(router)
app.mount('#app')