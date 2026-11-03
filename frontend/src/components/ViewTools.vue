<script setup lang="ts">
import type { ViewMode } from '@/scene/view'

defineProps<{ mode: ViewMode }>()
const emit = defineEmits<{ view: [mode: ViewMode] }>()
</script>

<template>
  <div class="tools" role="toolbar" aria-label="Camera">
    <button
      :class="{ active: mode === 'iso' }"
      title="3D view"
      aria-label="3D view"
      @click="emit('view', 'iso')"
    >
      <svg viewBox="0 0 16 16" aria-hidden="true">
        <path d="M8 1.5 14 5v6l-6 3.5L2 11V5z M2 5l6 3.5L14 5 M8 8.5v6" />
      </svg>
    </button>
    <button
      :class="{ active: mode === 'top' }"
      title="Top view"
      aria-label="Top view"
      @click="emit('view', 'top')"
    >
      <svg viewBox="0 0 16 16" aria-hidden="true">
        <path d="M2.5 2.5h11v11h-11z M2.5 8h11 M8 2.5v11" />
      </svg>
    </button>
  </div>
</template>

<style scoped>
.tools {
  position: absolute;
  top: 10px;
  right: 10px;
  display: flex;
  flex-direction: column;
  gap: 2px;
  padding: 3px;
  border: 1px solid var(--line-strong);
  border-radius: 4px;
  background: rgb(15 22 29 / 0.9);
}

button {
  display: grid;
  place-items: center;
  width: 28px;
  height: 28px;
  border: 1px solid transparent;
  border-radius: 3px;
  background: none;
  color: var(--text-dim);
}

button:hover {
  color: var(--text);
}

button.active {
  border-color: var(--accent-dim);
  color: var(--accent);
  background: rgb(34 211 238 / 0.08);
}

svg {
  width: 15px;
  height: 15px;
  fill: none;
  stroke: currentColor;
  stroke-width: 1.2;
  stroke-linejoin: round;
}
</style>
