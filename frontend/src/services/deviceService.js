import { extractNewFactor } from '../utils/fuel.js'

const API_BASE_URL = (import.meta.env.VITE_API_BASE_URL || '/api').replace(
  /\/$/,
  '',
)
const REQUEST_TIMEOUT = Number(import.meta.env.VITE_API_TIMEOUT_MS) || 12000

export class ApiError extends Error {
  constructor(message, status = 0, payload = null) {
    super(message)
    this.name = 'ApiError'
    this.status = status
    this.payload = payload
  }
}

function normalizeToken(token) {
  return token.trim().replace(/^Bearer\s+/i, '')
}

function getApiErrorMessage(status, payload) {
  const backendMessage =
    payload?.message || payload?.title || payload?.error || payload?.detail

  if (backendMessage) {
    return backendMessage
  }

  const messages = {
    400: 'Os dados enviados não foram aceitos. Confira os campos.',
    401: 'Sua sessão não é válida. Informe um token de acesso atualizado.',
    403: 'Você não tem permissão para recalibrar este veículo.',
    404: 'Dispositivo não encontrado. Confira o identificador.',
  }

  return messages[status] || 'Não foi possível concluir a recalibração.'
}

async function readResponse(response) {
  const text = await response.text()

  if (!text) {
    return null
  }

  try {
    return JSON.parse(text)
  } catch {
    return text
  }
}

export async function recalculateFuel({ deviceId, refueledGas, token }) {
  const controller = new AbortController()
  const timeout = window.setTimeout(() => controller.abort(), REQUEST_TIMEOUT)
  const headers = {
    Accept: 'application/json',
    'Content-Type': 'application/json',
  }

  if (token.trim()) {
    headers.Authorization = `Bearer ${normalizeToken(token)}`
  }

  try {
    const response = await fetch(
      `${API_BASE_URL}/devices/resetAndRecalculate?idDevice=${encodeURIComponent(deviceId)}`,
      {
        method: 'POST',
        headers,
        body: JSON.stringify({ refueledGas }),
        signal: controller.signal,
      },
    )
    const payload = await readResponse(response)

    if (!response.ok) {
      throw new ApiError(getApiErrorMessage(response.status, payload), response.status, payload)
    }

    return {
      payload,
      newFactor: extractNewFactor(payload),
    }
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new ApiError('A API demorou para responder. Tente novamente.')
    }

    if (error instanceof ApiError) {
      throw error
    }

    throw new ApiError(
      'Não foi possível conectar ao backend. Verifique a API e tente novamente.',
    )
  } finally {
    window.clearTimeout(timeout)
  }
}
