<script setup>
import { defineProps } from 'vue'
import { useViewModeStore } from '@/stores'
import { events } from './../DiagnosticsDialogue/dev.json'
import {
    getEventsByCategoryName,
    getSeverityColorForCategory,
} from '@/components/DiagnosticsDialogue/helpers'

const props = defineProps(['show'])

const viewModeStore = useViewModeStore()
const { editMode, toggleDiagnostics } = viewModeStore
</script>
<template>
    <div>
        <v-fab-transition leave-absolute>
            <v-btn
                icon
                color="primary"
                @click.stop="editMode()"
                v-show="props.show"
            >
                <v-icon>mdi-application-edit</v-icon>
            </v-btn>
        </v-fab-transition>
        <v-fab-transition leave-absolute>
            <v-btn
                icon
                color="primary"
                v-show="props.show"
            >
                <v-icon>mdi-cellphone-off</v-icon>
            </v-btn>
        </v-fab-transition>
        <v-fab-transition leave-absolute>
            <v-btn
                icon
                color="red"
                v-show="props.show"
            >
                <v-icon>mdi-microsoft-xbox-controller-off</v-icon>
            </v-btn>
        </v-fab-transition>
        <v-fab-transition leave-absolute>
            <v-btn
                icon
                color="primary"
                @click.stop="toggleDiagnostics()"
                v-show="props.show"
            >
                <v-badge
                    :color="getSeverityColorForCategory(events, 'all')"
                    :content="getEventsByCategoryName(events, 'all').length"
                >
                    <!-- @TODO: Connect with backend-->
                    <v-icon>mdi-alert</v-icon>
                </v-badge>
            </v-btn>
        </v-fab-transition>
    </div>
</template>
