# Frontend — Tela de combustível (FE-03)

Tela de combustível do OBD-II Retrofit.

## Executar localmente

```bash
npm install
npm run dev
```

Esta versão apresenta a calibração do combustível com os litros abastecidos e a
capacidade do tanque. O resultado é exibido localmente na tela, sem solicitar
identificador de veículo ou credenciais.

O backend disponível em `http://18.206.201.130:5000` exige autenticação Bearer
para consultar ou atualizar dispositivos. Por isso, a tela não faz chamadas
autenticadas enquanto não houver uma sessão válida definida pelo fluxo do produto.

## Verificações

```bash
npm test
npm run build
```
