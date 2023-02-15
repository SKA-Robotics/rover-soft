<script setup>
import { defineProps, onMounted, watch, ref, computed } from 'vue'
const props = defineProps(['extraConfig'])

const config = ref(JSON.parse(JSON.stringify(props.extraConfig)))
watch(props, () => {
    config.value = ref(JSON.parse(JSON.stringify(props.extraConfig)))
})
const topicName = computed(() => {
    return config.value.topicName
})

onMounted(() => {
    // console.log(config.value)
})
</script>
<template>
    <div>
        <h3>This will be chart ... soon</h3>
        <p>Topic name: {{ topicName }}</p>
    </div>
</template>
<style scoped>
div {
    display: flex;
    width: 100%;
    align-items: center;
    justify-content: center;
    background-color: black;
    flex-flow: column;
}

h3 {
    color: green;
}

p {
    color: green;
}
</style>
