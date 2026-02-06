import 'mdb-vue-ui-kit/css/mdb.min.css';

import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import router from './router'

import * as Icons from "oh-vue-icons/icons/";
import { addIcons } from 'oh-vue-icons';

const icons = Object.values({ ...Icons });
addIcons(...icons);

const app = createApp(App)

app.use(createPinia())
app.use(router)


app.mount('#app')
