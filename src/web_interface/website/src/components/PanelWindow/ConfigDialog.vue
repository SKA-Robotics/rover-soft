<script setup>
import { computed, defineProps, ref, watch } from 'vue'
import windows from '@/windows'
import configInputs from './configInputs'
const props = defineProps(['isOpen', 'config'])

const windowTypes = Object.entries(windows).map(([type, window]) => ({
    text: window.typeName,
    value: type,
}))
const configOptions = computed(() =>
    tempConfig.value.type in windows
        ? Object.entries(windows[tempConfig.value.type].configOptions).filter(
              ([, obj]) =>
                  !Array.isArray(obj) && !Array.isArray(obj.value)
                      ? typeof obj != 'function' && !obj.hide
                      : false
          )
        : []
)
const arrayConfigOptions = computed(() =>
    tempConfig.value.type in windows
        ? Object.entries(windows[tempConfig.value.type].configOptions)
              .filter(([name, obj]) => {
                  if (!Array.isArray(obj) && !Array.isArray(obj.value))
                      return false
                  if (obj.value) obj = obj.value
                  if (!tempConfig.value.extraConfig[name]) {
                      tempConfig.value.extraConfig[name] = Array(obj.length)
                          .fill(undefined)
                          .map(() => new Object())
                  }
                  while (tempConfig.value.extraConfig[name].length < obj.length)
                      tempConfig.value.extraConfig[name].push(new Object())
                  while (tempConfig.value.extraConfig[name].length > obj.length)
                      tempConfig.value.extraConfig[name].pop()
                  return true
              })
              .map(([name, array]) => [
                  name,
                  (array.value ? array.value : array).map((element) =>
                      Object.entries(element).filter(([, obj]) =>
                          !Array.isArray(obj)
                              ? typeof obj != 'function' && !obj.hide
                              : false
                      )
                  ),
              ])
        : []
)

const tempConfig = ref(JSON.parse(JSON.stringify(props.config)))
watch(props, () => {
    tempConfig.value = JSON.parse(JSON.stringify(props.config))
})

function updateValue(name, value, arrayName = undefined, index = undefined) {
    if (windows[tempConfig.value.type].configOptions.update)
        windows[tempConfig.value.type].configOptions.update(
            name,
            value,
            arrayName,
            index
        )
}
function scrollToBottom(arrayName) {
    setTimeout(
        () =>
            document
                .getElementById(`buttons ${arrayName}`)
                .scrollIntoView({ behavior: 'smooth' }),
        0.1
    )
}
</script>
<template>
    <v-dialog
        :value="props.isOpen"
        @input="$emit('close')"
        max-width="600px"
    >
        <v-card>
            <v-card-title>
                <span class="text-h5">Window configuration</span>
            </v-card-title>
            <v-card-text>
                <v-container>
                    <span class="text-subtitle1">General configuration</span>
                    <v-divider />
                    <v-container>
                        <v-select
                            v-model="tempConfig.type"
                            :items="windowTypes"
                            label="Type"
                            required
                        ></v-select>
                        <v-text-field
                            v-model="tempConfig.name"
                            label="Name"
                            required
                        ></v-text-field>
                    </v-container>

                    <span class="text-subtitle1">Additional configuration</span>
                    <v-divider />
                    <v-container>
                        <component
                            v-for="[name, options] in configOptions"
                            :key="name"
                            :is="configInputs[options.type]"
                            :configOptions="options"
                            v-model="tempConfig.extraConfig[name]"
                            @input="(value) => updateValue(name, value)"
                        ></component>
                        <v-list
                            v-for="[arrayName, array] in arrayConfigOptions"
                            :key="arrayName"
                            class="outer-frame"
                        >
                            <v-list-item
                                class="inner-frame"
                                v-for="(elements, index) in array"
                                :key="arrayName + index"
                            >
                                <component
                                    v-for="[name, options] in elements"
                                    :key="name + index"
                                    :is="configInputs[options.type]"
                                    :configOptions="options"
                                    v-model="
                                        tempConfig.extraConfig[arrayName][
                                            index
                                        ][name]
                                    "
                                    @input="
                                        (value) =>
                                            updateValue(
                                                name,
                                                value,
                                                arrayName,
                                                index
                                            )
                                    "
                                ></component>
                            </v-list-item>
                            <v-list-item
                                :id="'buttons ' + arrayName"
                                style="display: flex; justify-content: right"
                            >
                                <v-btn
                                    icon
                                    color="var(--v-miętowy-base)"
                                    height="4em"
                                    width="4em"
                                    @click="
                                        {
                                            updateValue('push', 1, arrayName)
                                            scrollToBottom(arrayName)
                                        }
                                    "
                                >
                                    <v-icon size="3em">mdi-plus-box</v-icon>
                                </v-btn>
                                <v-btn
                                    icon
                                    color="var(--v-morelowy-base)"
                                    height="4em"
                                    width="4em"
                                    @click="updateValue('pop', -1, arrayName)"
                                >
                                    <v-icon size="3em">mdi-minus-box</v-icon>
                                </v-btn>
                            </v-list-item>
                        </v-list>
                    </v-container>
                </v-container>
            </v-card-text>
            <v-card-actions>
                <v-spacer></v-spacer>
                <v-btn
                    color="primary darken-1"
                    text
                    @click="$emit('close', null)"
                >
                    Close
                </v-btn>
                <v-btn
                    color="primary darken-1"
                    @click="
                        $emit('close', JSON.parse(JSON.stringify(tempConfig)))
                    "
                >
                    Save
                </v-btn>
            </v-card-actions>
        </v-card>
    </v-dialog>
</template>
<style scoped>
.inner-frame {
    display: block;
    margin: 5px;
    background-color: #fafafa;
    border-color: gray;
    border-style: solid;
    border-width: 1px;
    border-radius: 10px;
}
.outer-frame {
    border-color: black;
    border-style: solid;
    border-width: 1.5px;
    border-radius: 15px;
}
</style>
