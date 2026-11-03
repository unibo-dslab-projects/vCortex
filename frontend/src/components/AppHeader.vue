<script setup lang="ts">
import { computed } from 'vue'
import { downloadUrl, type Format, type Status } from '@/services/api'

const props = defineProps<{ status: Status | null; reachable: boolean; format?: Format }>()

const source = computed(() => {
  const s = props.status
  if (!props.reachable || !s) return { on: false, text: 'Backend unreachable' }
  if (s.mode === 'offline') return { on: true, text: `Offline · ${s.source}` }
  return s.connected
    ? { on: true, text: `Live · ${s.source}` }
    : { on: false, text: 'Live · waiting for the robot' }
})
</script>

<template>
  <header class="bar">
    <div class="brand">
      <img src="/logo.svg" alt="" width="18" height="18" />
      <strong>vCortex</strong>
      <span class="hint">·</span>
      <span>Digital twin</span>
    </div>
    <span class="chip" :class="{ active: source.on }">
      <span class="dot" :class="{ on: source.on }" />{{ source.text }}
    </span>
    <span v-if="status?.run_id != null" class="hint run">
      run {{ status.run_id }} · seq {{ status.seq }}
    </span>
    <div class="spacer" />
    <a
      v-if="format"
      class="btn primary"
      :href="downloadUrl(format.id)"
      :aria-disabled="!status?.model"
      download
    >
      <svg viewBox="0 0 16 16" aria-hidden="true">
        <path d="M8 2v8m-3.5-3.5L8 10l3.5-3.5M3 13h10" />
      </svg>
      Export {{ format.label }}
    </a>
  </header>
</template>

<style scoped>
.bar {
  display: flex;
  align-items: center;
  gap: 14px;
  height: 48px;
  padding: 0 14px;
  border-bottom: 1px solid var(--line);
  background: var(--panel);
}

.brand {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  letter-spacing: 0.1em;
  text-transform: uppercase;
}

.spacer {
  flex: 1;
}

svg {
  width: 14px;
  height: 14px;
  fill: none;
  stroke: currentColor;
  stroke-width: 1.6;
  stroke-linecap: round;
  stroke-linejoin: round;
}

@media (max-width: 720px) {
  .run {
    display: none;
  }
}
</style>
