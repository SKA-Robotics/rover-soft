<script setup>
import { defineProps } from 'vue'
import { categories } from './dev.json'

const props = defineProps(['item'])
const { item } = props

function getIcon(category) {
    for (const cat of categories) {
        if ('name' in cat && 'icon' in cat) {
            if (cat.name.toLowerCase() === category.toLowerCase()) {
                return cat.icon
            }
        }
    }

    return 'mdi-package'
}

function formatTimestamp(timestamp) {
    return new Date(timestamp).toLocaleString()
}
</script>
<template>
    <v-expansion-panel :class="'event-' + item.level">
        <v-expansion-panel-header class="justify-start">
            <v-layout
                class="event-container"
                align-center
            >
                <v-col>
                    <v-icon>
                        {{ getIcon(item.category) }}
                    </v-icon>
                    {{ item.name }}
                </v-col>
                <v-col class="text-right">
                    {{ formatTimestamp(item.timestamp) }}
                </v-col>
            </v-layout>
        </v-expansion-panel-header>
        <v-expansion-panel-content>
            {{ item.message }}
            <v-col md="2">
                <v-simple-table v-if="item.values">
                    <thead>
                        <tr>
                            <th>Property</th>
                            <th>Value</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr
                            v-for="(key, value) in item.values"
                            :key="key"
                        >
                            <td>{{ value }}</td>
                            <td>{{ key }}</td>
                        </tr>
                    </tbody>
                </v-simple-table>
            </v-col>
        </v-expansion-panel-content>
    </v-expansion-panel>
</template>
<style scoped>
.v-expansion-panel {
    margin: 4px 0;
}
.event-container .v-icon {
    border-right: 1px solid #ddd;
    margin-right: 16px;
    padding-right: 16px;
}
.event-container .col {
    padding-top: 0;
    padding-bottom: 0;
}
.event-info .v-icon {
    color: var(--v-miętowy-base);
}
.event-warning .v-icon {
    color: var(--v-słoneczny-base);
}
.event-error .v-icon {
    color: var(--v-morelowy-base);
}
.event-stale .v-icon {
    color: var(--v-śliwkowy-base);
}
</style>
