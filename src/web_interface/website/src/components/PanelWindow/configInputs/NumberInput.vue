<script setup>
import { defineProps, computed } from 'vue'
const props = defineProps(['configOptions', 'value'])

const range = computed(() => {
    if (props.configOptions.range) return props.configOptions.range()
    else return {}
})
</script>
<template>
    <v-text-field
        :label="props.configOptions.name"
        :value="props.value"
        @input="(value) => $emit('input', value)"
        required
        type="number"
        :min="range.min"
        :max="range.max"
        :step="range.step"
    ></v-text-field>
</template>
