import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { ApiError, recalculateFuel } from './deviceService.js'

describe('recalculateFuel', () => {
  beforeEach(() => {
    vi.stubGlobal('window', {
      setTimeout,
      clearTimeout,
    })
  })

  afterEach(() => {
    vi.unstubAllGlobals()
    vi.restoreAllMocks()
  })

  it('envia o contrato esperado pelo endpoint de recálculo', async () => {
    const fetchMock = vi.fn().mockResolvedValue({
      ok: true,
      status: 200,
      text: vi.fn().mockResolvedValue('{"newFactor":1.2345}'),
    })
    vi.stubGlobal('fetch', fetchMock)

    const result = await recalculateFuel({
      deviceId: 'ESP32-A1/2',
      refueledGas: 4.4,
      token: 'Bearer jwt-de-teste',
    })

    expect(fetchMock).toHaveBeenCalledWith(
      '/api/devices/resetAndRecalculate?idDevice=ESP32-A1%2F2',
      expect.objectContaining({
        method: 'POST',
        headers: {
          Accept: 'application/json',
          Authorization: 'Bearer jwt-de-teste',
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ refueledGas: 4.4 }),
      }),
    )
    expect(result.newFactor).toBe(1.2345)
  })

  it('transforma a resposta 401 do backend em mensagem para a tela', async () => {
    vi.stubGlobal(
      'fetch',
      vi.fn().mockResolvedValue({
        ok: false,
        status: 401,
        text: vi.fn().mockResolvedValue(''),
      }),
    )

    await expect(
      recalculateFuel({
        deviceId: 'ESP32-TESTE',
        refueledGas: 4.4,
        token: '',
      }),
    ).rejects.toMatchObject({
      name: 'ApiError',
      status: 401,
      message: 'Sua sessão não é válida. Informe um token de acesso atualizado.',
    })
  })

  it('mantém os erros da API identificáveis pela tela', () => {
    expect(new ApiError('Falha', 400)).toBeInstanceOf(Error)
  })
})
