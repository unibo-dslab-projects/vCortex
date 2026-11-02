<script setup lang="ts">
import { computed, ref, watch } from 'vue'
import { putSettings, type Settings, type Status } from '@/services/api'

const props = defineProps<{ status: Status | null }>()
const settings = defineModel<Settings | null>('settings', { required: true })

const height = ref('')
const error = ref<string | null>(null)
const saving = ref(false)
let debounce: ReturnType<typeof setTimeout> | undefined

watch(
  () => settings.value?.wall_height,
  (value) => {
    if (value !== undefined && !saving.value) height.value = value.toFixed(2)
  },
  { immediate: true },
)

// one PUT per pause in typing: each one makes the backend run the model again
const onInput = () => {
  clearTimeout(debounce)
  debounce = setTimeout(save, 500)
}

async function save() {
  const value = Number(height.value)
  if (!Number.isFinite(value) || value === settings.value?.wall_height) return
  saving.value = true
  try {
    settings.value = await putSettings({ wall_height: value })
    error.value = null
  } catch (e) {
    error.value = (e as Error).message
  } finally {
    saving.value = false
  }
}

const model = computed(() => props.status?.model ?? null)
const kept = computed(() =>
  model.value ? Math.round((model.value.after_outliers / model.value.points_in) * 1000) / 10 : 0,
)
</script>

<template>
  <footer class="bar">
    <span v-if="model" class="item">
      <span class="dot on" />
      Model ready · {{ model.walls }} walls · {{ model.rooms }} rooms · {{ model.objects }} objects
      <span class="hint">· {{ Math.round(model.duration_ms) }} ms</span>
    </span>
    <span v-else class="item hint"><span class="dot" />No model run yet</span>

    <span v-if="model" class="item sep">
      Points kept: <b>{{ kept }}%</b>
      <span class="hint">
        ({{ model.after_outliers.toLocaleString('en') }} /
        {{ model.points_in.toLocaleString('en') }}) · tilt {{ model.tilt_deg.toFixed(1) }}° · yaw
        {{ model.yaw_deg.toFixed(1) }}°
      </span>
    </span>

    <div class="spacer" />

    <label class="item height">
      Wall height
      <input
        v-model="height"
        type="number"
        min="1.55"
        max="5.95"
        step="0.05"
        :disabled="!settings"
        :aria-invalid="!!error"
        @input="onInput"
      />
      m
      <span v-if="settings" class="hint">
        ({{ saving ? 'saving…' : settings.wall_height_source }})
      </span>
    </label>
    <span v-if="error" class="error" role="alert">{{ error }}</span>
  </footer>
</template>

<style scoped>
.bar {
  display: flex;
  align-items: center;
  gap: 16px;
  min-height: 36px;
  padding: 4px 14px;
  border-top: 1px solid var(--line);
  background: var(--panel);
  flex-wrap: wrap;
}

.item {
  display: inline-flex;
  align-items: center;
  gap: 6px;
}

.sep {
  padding-left: 16px;
  border-left: 1px solid var(--line-strong);
}

b {
  color: var(--accent);
  font-weight: 600;
}

.spacer {
  flex: 1;
}

input {
  width: 64px;
  padding: 3px 6px;
  border: 1px solid var(--line-strong);
  border-radius: 3px;
  background: var(--bg);
  color: var(--accent);
  text-align: right;
}

input:focus {
  outline: 1px solid var(--accent-dim);
}

input[aria-invalid='true'] {
  border-color: var(--bad);
}

.error {
  color: var(--bad);
}

@media (max-width: 720px) {
  .sep {
    display: none;
  }
}
</style>
