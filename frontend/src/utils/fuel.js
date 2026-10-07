const FACTOR_KEYS = [
  'newFactor',
  'fuelConsumptionFactor',
  'novoFator',
  'factor',
]

export class FuelValidationError extends Error {
  constructor(message) {
    super(message)
    this.name = 'FuelValidationError'
  }
}

export function parseFuelAmount(rawValue) {
  const value = String(rawValue ?? '').trim()

  if (!value) {
    throw new FuelValidationError('Informe quantos litros foram abastecidos.')
  }

  const normalized = value.includes(',')
    ? value.replace(/\./g, '').replace(',', '.')
    : value
  const amount = Number(normalized)

  if (!Number.isFinite(amount) || amount <= 0) {
    throw new FuelValidationError('Use um valor maior que zero, como 42,5.')
  }

  if (amount > 1000) {
    throw new FuelValidationError('Confira o valor informado em litros.')
  }

  return amount
}

export function extractNewFactor(payload) {
  if (typeof payload === 'number' && Number.isFinite(payload)) {
    return payload
  }

  if (!payload || typeof payload !== 'object') {
    return null
  }

  for (const key of FACTOR_KEYS) {
    const candidate = Number(payload[key])

    if (Number.isFinite(candidate)) {
      return candidate
    }
  }

  for (const nestedKey of ['data', 'result', 'device']) {
    const nestedFactor = extractNewFactor(payload[nestedKey])

    if (nestedFactor !== null) {
      return nestedFactor
    }
  }

  return null
}

function getPayloadObject(payload) {
  if (Array.isArray(payload)) {
    return payload[0] || null
  }

  if (!payload || typeof payload !== 'object') {
    return null
  }

  for (const key of ['data', 'device', 'result']) {
    const nested = payload[key]

    if (Array.isArray(nested)) {
      return nested[0] || null
    }

    if (nested && typeof nested === 'object') {
      return nested
    }
  }

  return payload
}

function toFiniteNumber(value) {
  if (value === null || value === undefined || value === '') {
    return null
  }

  const number = Number(value)
  return Number.isFinite(number) ? number : null
}

export function normalizeDeviceData(payload) {
  const device = getPayloadObject(payload)

  if (!device) {
    return null
  }

  return {
    gasolineLevel: toFiniteNumber(device.gasolineLevel),
    tankCapacity: toFiniteNumber(device.tankCapacity),
    fuelConsumptionFactor: toFiniteNumber(device.fuelConsumptionFactor),
    totalDistance: toFiniteNumber(device.totalDistance),
    tripConsumption: toFiniteNumber(device.tripConsumption),
    syncVersion: toFiniteNumber(device.sync_version ?? device.syncVersion),
  }
}

export function calculateFuelPercentage(gasolineLevel, tankCapacity) {
  if (
    !Number.isFinite(gasolineLevel) ||
    !Number.isFinite(tankCapacity) ||
    tankCapacity <= 0
  ) {
    return null
  }

  return Math.min(100, Math.max(0, (gasolineLevel / tankCapacity) * 100))
}

export function formatMetric(value, maximumFractionDigits = 2) {
  if (!Number.isFinite(value)) {
    return '--'
  }

  return new Intl.NumberFormat('pt-BR', {
    minimumFractionDigits: 0,
    maximumFractionDigits,
  }).format(value)
}

export function formatFuelFactor(value) {
  return new Intl.NumberFormat('pt-BR', {
    minimumFractionDigits: 2,
    maximumFractionDigits: 4,
  }).format(value)
}
