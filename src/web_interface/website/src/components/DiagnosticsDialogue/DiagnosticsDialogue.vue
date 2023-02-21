<script setup>
import { computed, ref } from 'vue'
import { useViewModeStore } from '@/stores'
import { categories, events } from './dev.json'
import DiagnosticsEvent from '@/components/DiagnosticsDialogue/DiagnosticsEvent.vue'
import {
    getEventsByCategory,
    getEventsByCategoryName,
    getSeverityColorForCategory,
} from '@/components/DiagnosticsDialogue/helpers'

const viewModeStore = useViewModeStore()
const { toggleDiagnostics } = viewModeStore

const sortedEvents = events.sort((a, b) => b.timestamp - a.timestamp)
const selectedCategory = ref(0)
const displayEvents = computed(() => {
    const category = categories[selectedCategory.value]
    return getEventsByCategory(sortedEvents, category)
})
</script>
<template>
    <v-dialog
        v-model="viewModeStore.diagnostics"
        transition="dialog-bottom-transition"
    >
        <template>
            <v-card>
                <v-toolbar
                    color="primary"
                    dark
                >
                    Diagnostics
                    <v-col class="text-right">
                        <v-btn
                            text
                            @click.stop="toggleDiagnostics()"
                        >
                            <v-icon>mdi-close</v-icon>
                        </v-btn>
                    </v-col>
                </v-toolbar>
                <v-card-text>
                    <v-layout row>
                        <v-col
                            md="2"
                            class="left-column"
                        >
                            <v-list
                                nav
                                light
                            >
                                <v-list-item-group
                                    v-model="selectedCategory"
                                    color="primary"
                                >
                                    <v-list-item
                                        v-for="(item, i) in categories"
                                        :key="i"
                                    >
                                        <v-list-item-icon>
                                            <v-badge
                                                :content="
                                                    getEventsByCategoryName(
                                                        sortedEvents,
                                                        item.name.toLowerCase()
                                                    ).length
                                                "
                                                :color="
                                                    getSeverityColorForCategory(
                                                        events,
                                                        item.name.toLowerCase()
                                                    )
                                                "
                                            >
                                                <v-icon>{{ item.icon }}</v-icon>
                                            </v-badge>
                                        </v-list-item-icon>
                                        <v-list-item-title>
                                            {{ item.name }}
                                        </v-list-item-title>
                                    </v-list-item>
                                </v-list-item-group>
                            </v-list>
                        </v-col>
                        <v-col
                            md="10"
                            class="right-column"
                        >
                            <v-expansion-panels
                                light
                                multiple
                                class="events-list"
                            >
                                <DiagnosticsEvent
                                    v-for="item in displayEvents"
                                    :key="item"
                                    :item="item"
                                />
                            </v-expansion-panels>
                        </v-col>
                    </v-layout>
                </v-card-text>
            </v-card>
        </template>
    </v-dialog>
</template>
<style scoped>
.left-column {
    border-right: 1px solid #eee;
}

.right-column {
    margin: 16px 0;
}

.events-list {
    overflow: auto;
    max-height: 100%;
}
</style>
