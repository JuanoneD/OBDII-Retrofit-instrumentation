import { useState } from 'react'

function parseNumber(value) {
  return Number.parseFloat(String(value).replace(',', '.'))
}

export default function FuelPage() {
  const [refueledGas, setRefueledGas] = useState('')
  const [tankCapacity, setTankCapacity] = useState('')
  const [summary, setSummary] = useState(null)
  const [error, setError] = useState('')

  function handleSubmit(event) {
    event.preventDefault()
    const added = parseNumber(refueledGas)
    const capacity = parseNumber(tankCapacity)

    if (!Number.isFinite(added) || added <= 0) {
      setError('Informe quantos litros foram abastecidos.')
      return
    }

    if (!Number.isFinite(capacity) || capacity <= 0) {
      setError('Informe a capacidade do tanque.')
      return
    }

    setError('')
    setSummary({ added, capacity })
  }

  if (summary) {
    return (
      <main className="fuel-screen fuel-summary-screen">
        <header className="screen-header">
          <p>OBD-II RETROFIT</p>
          <h1>Combustível</h1>
          <span>Resumo da calibração</span>
        </header>

        <section className="fuel-result-hero">
          <span>Combustível estimado</span>
          <strong>{summary.capacity.toFixed(1).replace('.', ',')} L</strong>
          <small>Capacidade informada do tanque</small>
        </section>

        <section className="fuel-result-grid" aria-label="Informações de combustível">
          <article>
            <span>Último abastecimento</span>
            <strong>{summary.added.toFixed(1).replace('.', ',')} L</strong>
          </article>
          <article>
            <span>Média de consumo</span>
            <strong>— km/L</strong>
          </article>
          <article>
            <span>Autonomia estimada</span>
            <strong>— km</strong>
          </article>
          <article>
            <span>Status</span>
            <strong className="result-ok">Calibrado</strong>
          </article>
        </section>

        <button className="secondary-button" type="button" onClick={() => setSummary(null)}>
          NOVA CALIBRAÇÃO
        </button>
      </main>
    )
  }

  return (
    <main className="fuel-screen">
      <header className="screen-header">
        <p>OBD-II RETROFIT</p>
        <h1>Combustível</h1>
        <span>Calibre o tanque informando o abastecimento.</span>
      </header>

      <form className="fuel-card fuel-calibration-card" onSubmit={handleSubmit} noValidate>
        <div className="fuel-card-icon">⛽</div>
        <h2>Calibrar combustível</h2>
        <p className="fuel-card-description">Essas informações ajudam a estimar o combustível disponível e o consumo do veículo.</p>

        <label htmlFor="refueled-gas">Quanto você abasteceu?</label>
        <div className="liters-field">
          <input
            id="refueled-gas"
            type="text"
            inputMode="decimal"
            placeholder="Ex.: 42,5"
            value={refueledGas}
            onChange={(event) => setRefueledGas(event.target.value)}
          />
          <span>litros</span>
        </div>

        <label htmlFor="tank-capacity">Qual é a capacidade do tanque?</label>
        <div className="liters-field">
          <input
            id="tank-capacity"
            type="text"
            inputMode="decimal"
            placeholder="Ex.: 45"
            value={tankCapacity}
            onChange={(event) => setTankCapacity(event.target.value)}
          />
          <span>litros</span>
        </div>

        {error && <div className="feedback feedback-error" role="alert">{error}</div>}

        <button className="primary-button" type="submit">
          CALIBRAR COMBUSTÍVEL
        </button>
      </form>
    </main>
  )
}
