import { useEffect, useRef, useState } from 'react';
import { localize, useLocaleStore } from '../stores/localeStore';

export function formatValue(value: number): string {
  if (!Number.isFinite(value)) return '—';
  if (Math.abs(value) < 1e-10) return '0';
  return String(Number(value.toPrecision(8)) || 0);
}

interface NumberFieldProps {
  label: string;
  value: number;
  onCommit: (value: number) => number | void;
  disabled?: boolean;
  min?: number;
  max?: number;
  step?: number;
  scrub?: boolean;
  unit?: string;
  id?: string;
}

export function NumberField({ label, value, onCommit, disabled, min, max, step = 1, scrub = false, unit, id }: NumberFieldProps) {
  const language = useLocaleStore((state) => state.language);
  const [draft, setDraft] = useState(formatValue(value));
  const inputRef = useRef<HTMLInputElement>(null);
  const dragRef = useRef<{ pointerId: number; startX: number; startValue: number; moved: boolean } | null>(null);
  useEffect(() => {
    if (document.activeElement !== inputRef.current || dragRef.current?.moved) setDraft(formatValue(value));
  }, [value]);

  const commit = (raw: string) => {
    const parsed = Number(raw.replace('，', '.'));
    if (!Number.isFinite(parsed)) { setDraft(formatValue(value)); return; }
    const result = onCommit(parsed);
    setDraft(formatValue(typeof result === 'number' ? result : parsed));
  };

  return (
    <label className={`number-field ${disabled ? 'is-disabled' : ''} ${scrub ? 'is-scrubbable' : ''}`}>
      <span>{label}</span>
      <input
        id={id} ref={inputRef} type="text" inputMode="decimal" value={draft} disabled={disabled}
        aria-label={`${label}${unit ? ` (${unit})` : ''}`}
        data-min={min} data-max={max}
        title={scrub ? localize(language, '左右拖动调节；点击后可输入精确数值', 'Drag left or right to adjust; click to enter an exact value') : undefined}
        onChange={(event) => setDraft(event.target.value)}
        onBlur={() => { if (!dragRef.current?.moved) commit(draft); }}
        onKeyDown={(event) => {
          if (event.key === 'Enter') event.currentTarget.blur();
          if (event.key === 'Escape') { setDraft(formatValue(value)); event.currentTarget.blur(); }
        }}
        onPointerDown={(event) => {
          if (!scrub || disabled || event.button !== 0) return;
          dragRef.current = { pointerId: event.pointerId, startX: event.clientX, startValue: value, moved: false };
          event.currentTarget.setPointerCapture(event.pointerId);
        }}
        onPointerMove={(event) => {
          const drag = dragRef.current;
          if (!drag || drag.pointerId !== event.pointerId) return;
          const delta = event.clientX - drag.startX;
          if (!drag.moved && Math.abs(delta) < 3) return;
          drag.moved = true;
          const next = drag.startValue + delta * step * (event.shiftKey ? 0.1 : 1);
          const result = onCommit(next);
          setDraft(formatValue(typeof result === 'number' ? result : next));
        }}
        onPointerUp={(event) => {
          if (dragRef.current?.pointerId !== event.pointerId) return;
          dragRef.current = null;
          if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
          setDraft(formatValue(value));
        }}
        onPointerCancel={() => { dragRef.current = null; setDraft(formatValue(value)); }}
      />
      {unit && <small>{unit}</small>}
    </label>
  );
}
