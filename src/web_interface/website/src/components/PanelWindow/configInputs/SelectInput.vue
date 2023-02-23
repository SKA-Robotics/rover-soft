<script setup>
import { computed, defineProps } from 'vue'
const props = defineProps(['configOptions', 'value'])

const possibleValues = computed(() => {
    if (props.configOptions.possibleValues)
        return props.configOptions.possibleValues()
    else return []
})
</script>
<template>
    <v-select
        :label="props.configOptions.name"
        :value="props.value"
        @input="(value) => $emit('input', value)"
        :items="possibleValues"
        required
    ></v-select>
</template>
