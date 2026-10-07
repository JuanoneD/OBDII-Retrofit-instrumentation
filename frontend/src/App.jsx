import { useEffect, useMemo, useState } from 'react'
import {
  ApiError,
  getDevice,
  recalculateFuel,
} from './services/deviceService.js'
import {
  calculateFuelPercentage,
  FuelValidationError,
  formatFuelFactor,
  formatMetric,
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
  const [device, setDevice] = useState(null)
  const [deviceStatus, setDeviceStatus] = useState(null)
  const [isLoadingDevice, setIsLoadingDevice] = useState(false)
  const [isSubmitting, setIsSubmitting] = useState(false)
  const [feedback, setFeedback] = useState(null)

  const fuelPercentage = useMemo(
    () =>
      calculateFuelPercentage(device?.gasolineLevel, device?.tankCapacity),
    [device],
  )

  function saveAccessSettings() {
    const normalizedDeviceId = deviceId.trim()
    const normalizedToken = token.trim()

    if (normalizedDeviceId) {
      window.localStorage.setItem(DEVICE_STORAGE_KEY, normalizedDeviceId)
    } else {
      window.localStorage.removeItem(DEVICE_STORAGE_KEY)
    }

    if (normalizedToken) {
      window.sessionStorage.setItem(TOKEN_STORAGE_KEY, normalizedToken)
    } else {
      window.sessionStorage.removeItem(TOKEN_STORAGE_KEY)
    }
  }

  async function loadDeviceData({ silent = false } = {}) {
    const normalizedDeviceId = deviceId.trim()

    if (!normalizedDeviceId) {
      setDevice(null)
      setDeviceStatus({
        type: 'error',
        message: 'Configure o identificador do veículo para carregar os dados.',
      })
      return false
    }

    if (!silent) {
      setDeviceStatus(null)
    }

    setIsLoadingDevice(true)
    saveAccessSettings()

    try {
      const result = await getDevice({
        deviceId: normalizedDeviceId,
        token,
      })

      setDevice(result.device)
      setDeviceStatus(
        result.device
          ? { type: 'success', message: 'Dados atualizados.' }
          : { type: 'error', message: 'A API não retornou dados do veículo.' },
      )
      return Boolean(result.device)
    } catch (error) {
      setDevice(null)
      setDeviceStatus({
        type: 'error',
        message:
          error instanceof ApiError
            ? error.message
            : 'Não foi possível carregar o veículo.',
      })
      return false
    } finally {
      setIsLoadingDevice(false)
    }
  }

  useEffect(() => {
    if (deviceId.trim()) {
      loadDeviceData({ silent: true })
    }
    // A carga inicial deve usar somente os valores restaurados do navegador.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  async function handleSubmit(event) {
    event.preventDefault()
    setFeedback(null)

    const normalizedDeviceId = deviceId.trim()

    if (!normalizedDeviceId) {
      setFeedback({
        type: 'error',
        message: 'Configure o identificador do veículo antes de recalibrar.',
      })
      return
    }

    try {
      const refueledGas = parseFuelAmount(liters)
      setIsSubmitting(true)
      saveAccessSettings()

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
      await loadDeviceData({ silent: true })
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
        <span className="vehicle-label">
          {deviceId.trim() ? `Veículo: ${deviceId.trim()}` : 'Nenhum veículo configurado'}
        </span>
      </header>

      <section className="fuel-reading" aria-label="Resumo do combustível">
        <div className="reading-heading">
          <h2>Combustível estimado</h2>
          <button
            className="refresh-button"
            type="button"
            onClick={() => loadDeviceData()}
            disabled={isLoadingDevice}
          >
            {isLoadingDevice ? 'Atualizando...' : 'Atualizar'}
          </button>
        </div>

        <strong className="fuel-value">
          {formatMetric(device?.gasolineLevel)} <small>L</small>
        </strong>
        <span className="fuel-percentage">
          {fuelPercentage === null
            ? 'Aguardando dados do veículo'
            : `${formatMetric(fuelPercentage, 1)}% do tanque`}
        </span>

        <div className="fuel-progress" aria-hidden="true">
          <span style={{ width: `${fuelPercentage ?? 0}%` }} />
        </div>

        {deviceStatus && (
          <p className={`data-status data-status-${deviceStatus.type}`}>
            {deviceStatus.message}
          </p>
        )}
      </section>

      <section className="vehicle-data" aria-label="Dados atuais do veículo">
        <div>
          <span>Capacidade do tanque</span>
          <strong>{formatMetric(device?.tankCapacity)} L</strong>
        </div>
        <div>
          <span>Fator atual</span>
          <strong>{formatMetric(device?.fuelConsumptionFactor, 4)}</strong>
        </div>
        <div>
          <span>Consumo da viagem</span>
          <strong>{formatMetric(device?.tripConsumption)} L</strong>
        </div>
        <div>
          <span>Distância total</span>
          <strong>{formatMetric(device?.totalDistance)} km</strong>
        </div>
      </section>

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
          <summary>Configuração temporária do veículo</summary>
          <div className="settings-content">
            <label htmlFor="device-id">Identificador do veículo</label>
            <input
              id="device-id"
              name="deviceId"
              type="text"
              autoComplete="off"
              placeholder="Ex.: ESP32-A1B2C3D4"
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

            <button
              className="load-button"
              type="button"
              onClick={() => loadDeviceData()}
              disabled={isLoadingDevice}
            >
              {isLoadingDevice ? 'CARREGANDO...' : 'SALVAR E CARREGAR DADOS'}
            </button>
          </div>
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

        <button className="submit-button" type="submit" disabled={isSubmitting}>
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
