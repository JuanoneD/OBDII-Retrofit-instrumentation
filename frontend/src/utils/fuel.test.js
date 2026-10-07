import { describe, expect, it } from 'vitest'
import {
  calculateFuelPercentage,
  FuelValidationError,
  extractNewFactor,
  formatMetric,
  normalizeDeviceData,
  parseFuelAmount,
} from './fuel.js'

describe('parseFuelAmount', () => {
  it('aceita separador decimal brasileiro', () => {
    expect(parseFuelAmount('42,5')).toBe(42.5)
  })

  it('aceita separador decimal internacional', () => {
    expect(parseFuelAmount('18.75')).toBe(18.75)
  })

  it('rejeita valor vazio ou não positivo', () => {
    expect(() => parseFuelAmount('')).toThrow(FuelValidationError)
    expect(() => parseFuelAmount('0')).toThrow(FuelValidationError)
  })
})

describe('extractNewFactor', () => {
  it('encontra o novo fator na resposta direta', () => {
    expect(extractNewFactor({ newFactor: 1.2345 })).toBe(1.2345)
  })

  it('encontra o fator em respostas aninhadas', () => {
    expect(
      extractNewFactor({ data: { fuelConsumptionFactor: 0.9876 } }),
    ).toBe(0.9876)
  })

  it('retorna null quando o backend não envia o fator', () => {
    expect(extractNewFactor({ message: 'ok' })).toBeNull()
  })
})

describe('normalizeDeviceData', () => {
  it('normaliza os campos retornados pelo endpoint de dispositivo', () => {
    expect(
      normalizeDeviceData({
        data: {
          gasolineLevel: 32.7,
          tankCapacity: 45,
          fuelConsumptionFactor: 1.15,
          totalDistance: 10234.5,
          tripConsumption: 8.2,
          sync_version: 3,
        },
      }),
    ).toEqual({
      gasolineLevel: 32.7,
      tankCapacity: 45,
      fuelConsumptionFactor: 1.15,
      totalDistance: 10234.5,
      tripConsumption: 8.2,
      syncVersion: 3,
    })
  })

  it('aceita uma lista de dispositivos e mantém ausências como null', () => {
    expect(normalizeDeviceData([{ gasolineLevel: 10 }])).toEqual({
      gasolineLevel: 10,
      tankCapacity: null,
      fuelConsumptionFactor: null,
      totalDistance: null,
      tripConsumption: null,
      syncVersion: null,
    })
  })
})

describe('calculateFuelPercentage', () => {
  it('calcula e limita o percentual do tanque', () => {
    expect(calculateFuelPercentage(32.7, 45)).toBeCloseTo(72.67)
    expect(calculateFuelPercentage(60, 45)).toBe(100)
  })

  it('não calcula sem capacidade válida', () => {
    expect(calculateFuelPercentage(10, 0)).toBeNull()
    expect(calculateFuelPercentage(null, 45)).toBeNull()
  })
})

describe('formatMetric', () => {
  it('formata números e usa marcador quando não há dado', () => {
    expect(formatMetric(32.7)).toBe('32,7')
    expect(formatMetric(null)).toBe('--')
  })
})
