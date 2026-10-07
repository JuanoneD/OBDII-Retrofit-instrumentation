import { describe, expect, it } from 'vitest'
import {
  FuelValidationError,
  extractNewFactor,
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
