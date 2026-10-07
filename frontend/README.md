# Frontend — Tela de combustível (FE-03)

Primeira entrega do frontend do OBD-II Retrofit, limitada à tela de recálculo do
fator de consumo.

## Executar localmente

```bash
npm install
npm run dev
```

O Vite encaminha `/api` para `http://18.206.201.130:5000`, evitando problemas de
CORS durante o desenvolvimento local.

## Endpoint utilizado

```http
POST /devices/resetAndRecalculate?idDevice={idDevice}
Authorization: Bearer {token}
Content-Type: application/json

{
  "refueledGas": 42.5
}
```

O backend é responsável por calcular e persistir o novo fator. O frontend apenas
valida a entrada, envia os litros abastecidos e apresenta a resposta.

O identificador do dispositivo é mantido no `localStorage`. O token JWT fica
somente no `sessionStorage`, sendo removido ao encerrar a sessão do navegador.

## Verificações

```bash
npm test
npm run build
```
