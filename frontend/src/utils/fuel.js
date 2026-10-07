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

export function formatFuelFactor(value) {
  return new Intl.NumberFormat('pt-BR', {
    minimumFractionDigits: 2,
    maximumFractionDigits: 4,
  }).format(value)
}
