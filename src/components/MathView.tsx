import katex from 'katex';
import type { Matrix4 } from 'three';
import { matrixRows } from '../math/transform';
import { formatValue } from './NumberField';

export function Formula({ tex, display = false }: { tex: string; display?: boolean }) {
  return <span className="formula" dangerouslySetInnerHTML={{ __html: katex.renderToString(tex, { throwOnError: false, displayMode: display }) }} />;
}

export function MatrixView({ matrix, size = 4, label, compact = false }: { matrix: Matrix4; size?: 3 | 4; label: string; compact?: boolean }) {
  const rows = matrixRows(matrix, size);
  return (
    <div className={`matrix-card ${compact ? 'compact' : ''}`}>
      <div className="matrix-title"><Formula tex={label} /></div>
      <div className="matrix-bracket"><div className="matrix-grid" style={{ gridTemplateColumns: `repeat(${size}, minmax(0, 1fr))` }}>
        {rows.flat().map((number, index) => <span key={index} className={size === 4 && index % 4 === 3 && index !== 15 ? 'matrix-translation' : ''}>{formatValue(Number(number.toFixed(3)))}</span>)}
      </div></div>
    </div>
  );
}

export function VectorReadout({ label, values, unit = '' }: { label: string; values: number[]; unit?: string }) {
  return <div className="vector-readout"><span>{label}</span><strong>[ {values.map((value) => `${formatValue(value)}${unit}`).join('  ·  ')} ]</strong></div>;
}
