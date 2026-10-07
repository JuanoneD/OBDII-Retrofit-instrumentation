import { useState } from 'react'
import { ApiError, recalculateFuel } from './services/deviceService.js'
import {
  FuelValidationError,
  formatFuelFactor,
  parseFuelAmount,
} from './utils/fuel.js'

const DEVICE_STORAGE_KEY = 'obdii.deviceId'
const TOKEN_STORAGE_KEY = 'obdii.authToken'

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

  async function handleSubmit(event) {
    event.preventDefault()
    setFeedback(null)

    const normalizedDeviceId = deviceId.trim()

    if (!normalizedDeviceId) {
      setFeedback({
        type: 'error',
        message: 'Informe o identificador do veículo.',
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
            ? 'Recalibração concluída e salva.'
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
    <main className="fuel-screen">
      <header className="screen-header">
        <p>OBD-II RETROFIT</p>
        <h1>Calibração de Combustível</h1>
        <span>Informe os litros adicionados no último abastecimento.</span>
      </header>

      <form className="calibration-form" onSubmit={handleSubmit} noValidate>
        <h2>Calibrar consumo</h2>

        <label htmlFor="liters">Quanto de gasolina entrou no posto?</label>
        <div className="liters-field">
          <input
            id="liters"
            name="liters"
            type="text"
            inputMode="decimal"
            autoComplete="off"
            placeholder="Ex.: 4,40"
            value={liters}
            onChange={(event) => setLiters(event.target.value)}
          />
          <span>litros</span>
        </div>

        <details>
          <summary>Configuração temporária de acesso</summary>
          <label htmlFor="device-id">Identificador do veículo</label>
          <input
            id="device-id"
            name="deviceId"
            type="text"
            autoComplete="off"
            placeholder="Ex.: A1B2C3D4E5F6"
            value={deviceId}
            onChange={(event) => setDeviceId(event.target.value)}
          />

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
        </details>

        {feedback && (
          <div
            className={`feedback feedback-${feedback.type}`}
            role={feedback.type === 'error' ? 'alert' : 'status'}
            aria-live="polite"
          >
            <strong>
              {feedback.type === 'success'
                ? 'Abastecimento registrado'
                : 'Não foi possível recalibrar'}
            </strong>
            <p>{feedback.message}</p>
            {feedback.factor !== null && feedback.factor !== undefined && (
              <p>
                Novo fator: <b>{formatFuelFactor(feedback.factor)}</b>
              </p>
            )}
          </div>
        )}

        <button type="submit" disabled={isSubmitting}>
          {isSubmitting ? 'RECALCULANDO...' : 'CALCULAR E ATUALIZAR'}
        </button>
      </form>

      <p className="screen-note">
        O cálculo do novo fator e o salvamento são realizados pelo backend.
      </p>
    </main>
  )
}

export default App
