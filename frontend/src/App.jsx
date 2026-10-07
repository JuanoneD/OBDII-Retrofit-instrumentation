import { useMemo, useState } from 'react'
import { ApiError, recalculateFuel } from './services/deviceService.js'
import {
  FuelValidationError,
  formatFuelFactor,
  parseFuelAmount,
} from './utils/fuel.js'

const DEVICE_STORAGE_KEY = 'obdii.deviceId'
const TOKEN_STORAGE_KEY = 'obdii.authToken'

function FuelIcon() {
  return (
    <svg viewBox="0 0 24 24" aria-hidden="true">
      <path d="M6 3h8a2 2 0 0 1 2 2v15H4V5a2 2 0 0 1 2-2Z" />
      <path d="M7 6h6v5H7zM16 8h2l2 2v7a2 2 0 0 1-4 0M2 20h16" />
    </svg>
  )
}

function CheckIcon() {
  return (
    <svg viewBox="0 0 24 24" aria-hidden="true">
      <path d="m5 12 4 4L19 6" />
    </svg>
  )
}

function ArrowIcon() {
  return (
    <svg viewBox="0 0 24 24" aria-hidden="true">
      <path d="M5 12h14M13 6l6 6-6 6" />
    </svg>
  )
}

function App() {
  const [deviceId, setDeviceId] = useState(
    () => window.localStorage.getItem(DEVICE_STORAGE_KEY) || '',
  )
  const [token, setToken] = useState(
    () => window.sessionStorage.getItem(TOKEN_STORAGE_KEY) || '',
  )
  const [liters, setLiters] = useState('')
  const [isSubmitting, setIsSubmitting] = useState(false)
  const [feedback, setFeedback] = useState(null)

  const apiLabel = useMemo(() => {
    const configuredUrl = import.meta.env.VITE_API_BASE_URL || '/api'
    return configuredUrl === '/api' ? 'API conectada por proxy seguro' : 'API configurada'
  }, [])

  async function handleSubmit(event) {
    event.preventDefault()
    setFeedback(null)

    const normalizedDeviceId = deviceId.trim()

    if (!normalizedDeviceId) {
      setFeedback({
        type: 'error',
        message: 'Informe o identificador do dispositivo antes de continuar.',
      })
      return
    }

    try {
      const refueledGas = parseFuelAmount(liters)
      setIsSubmitting(true)

      window.localStorage.setItem(DEVICE_STORAGE_KEY, normalizedDeviceId)

      if (token.trim()) {
        window.sessionStorage.setItem(TOKEN_STORAGE_KEY, token.trim())
      } else {
        window.sessionStorage.removeItem(TOKEN_STORAGE_KEY)
      }

      const result = await recalculateFuel({
        deviceId: normalizedDeviceId,
        refueledGas,
        token,
      })

      setFeedback({
        type: 'success',
        factor: result.newFactor,
        message:
          result.newFactor === null
            ? 'Recalibração concluída e salva pelo backend.'
            : 'Novo fator calculado e salvo com sucesso.',
      })
      setLiters('')
    } catch (error) {
      const message =
        error instanceof FuelValidationError || error instanceof ApiError
          ? error.message
          : 'Ocorreu um erro inesperado. Tente novamente.'

      setFeedback({ type: 'error', message })
    } finally {
      setIsSubmitting(false)
    }
  }

  return (
    <div className="app-shell">
      <header className="topbar">
        <a className="brand" href="/" aria-label="OBD-II Retrofit — início">
          <span className="brand-mark">
            <FuelIcon />
          </span>
          <span>
            <strong>OBD-II</strong>
            <small>Retrofit</small>
          </span>
        </a>

        <div className="api-status">
          <span aria-hidden="true" />
          {apiLabel}
        </div>
      </header>

      <main>
        <section className="intro-panel" aria-labelledby="page-title">
          <div className="intro-content">
            <span className="card-code">FE-03 · COMBUSTÍVEL</span>
            <h1 id="page-title">
              Abasteceu? <em>Recalibre</em> em poucos segundos.
            </h1>
            <p className="intro-copy">
              Informe a quantidade adicionada ao tanque. O backend compara o
              abastecimento real com o consumo medido e atualiza o fator do seu
              veículo.
            </p>

            <ol className="steps" aria-label="Etapas da recalibração">
              <li>
                <span>1</span>
                <div>
                  <strong>Identifique o veículo</strong>
                  <small>Use o ID vinculado ao módulo ESP32.</small>
                </div>
              </li>
              <li>
                <span>2</span>
                <div>
                  <strong>Informe o abastecimento</strong>
                  <small>Digite exatamente os litros mostrados na bomba.</small>
                </div>
              </li>
              <li>
                <span>3</span>
                <div>
                  <strong>Confirme o novo fator</strong>
                  <small>O cálculo e a persistência acontecem no backend.</small>
                </div>
              </li>
            </ol>
          </div>

          <div className="formula-card">
            <span>FÓRMULA APLICADA PELA API</span>
            <p>
              novo fator = fator anterior ×
              <strong> litros abastecidos / litros calculados</strong>
            </p>
          </div>
        </section>

        <section className="form-panel" aria-labelledby="form-title">
          <div className="form-heading">
            <div className="form-icon">
              <FuelIcon />
            </div>
            <div>
              <span>RECALIBRAÇÃO</span>
              <h2 id="form-title">Registrar abastecimento</h2>
            </div>
          </div>

          <form onSubmit={handleSubmit} noValidate>
            <label htmlFor="device-id">Identificador do dispositivo</label>
            <input
              id="device-id"
              name="deviceId"
              type="text"
              autoComplete="off"
              placeholder="Ex.: ESP32-A1B2C3D4"
              value={deviceId}
              onChange={(event) => setDeviceId(event.target.value)}
              aria-describedby="device-help"
            />
            <small id="device-help" className="field-help">
              O identificador fica salvo apenas neste navegador.
            </small>

            <label htmlFor="liters">Litros adicionados</label>
            <div className="input-with-unit">
              <input
                id="liters"
                name="liters"
                type="text"
                inputMode="decimal"
                autoComplete="off"
                placeholder="0,00"
                value={liters}
                onChange={(event) => setLiters(event.target.value)}
                aria-describedby="liters-help"
              />
              <span>litros</span>
            </div>
            <small id="liters-help" className="field-help">
              Você pode usar vírgula ou ponto para os centavos.
            </small>

            <details className="access-details">
              <summary>Configuração de acesso à API</summary>
              <div className="access-content">
                <label htmlFor="token">Token JWT</label>
                <textarea
                  id="token"
                  name="token"
                  rows="3"
                  autoComplete="off"
                  spellCheck="false"
                  placeholder="Cole somente o token de acesso"
                  value={token}
                  onChange={(event) => setToken(event.target.value)}
                />
                <small className="field-help">
                  Usado temporariamente até a tela de login ficar pronta. O token
                  não é salvo permanentemente.
                </small>
              </div>
            </details>

            {feedback && (
              <div
                className={`feedback feedback-${feedback.type}`}
                role={feedback.type === 'error' ? 'alert' : 'status'}
                aria-live="polite"
              >
                <span className="feedback-icon">
                  {feedback.type === 'success' ? <CheckIcon /> : '!'}
                </span>
                <div>
                  <strong>
                    {feedback.type === 'success'
                      ? 'Abastecimento registrado'
                      : 'Não foi possível continuar'}
                  </strong>
                  <p>{feedback.message}</p>
                  {feedback.factor !== null && feedback.factor !== undefined && (
                    <p className="factor-result">
                      Novo fator: <b>{formatFuelFactor(feedback.factor)}</b>
                    </p>
                  )}
                </div>
              </div>
            )}

            <button className="submit-button" type="submit" disabled={isSubmitting}>
              <span>{isSubmitting ? 'Recalculando...' : 'Recalcular fator'}</span>
              {isSubmitting ? <i className="spinner" /> : <ArrowIcon />}
            </button>
          </form>

          <p className="security-note">
            <span aria-hidden="true">✓</span>
            O cálculo é realizado e persistido pelo backend.
          </p>
        </section>
      </main>

      <footer>
        <span>OBD-II Retrofit</span>
        <span>Telemetria veicular · TCC 2026</span>
      </footer>
    </div>
  )
}

export default App
