# Sistema de Navegação Inercial 1D - Visão Geral

## Objetivo

Este projeto implementa um **Sistema de Navegação Inercial (INS) em 1 dimensão** para fins de estudo acadêmico (IC/TCC). O foco é a clareza conceitual, organização modular e rastreabilidade entre teoria matemática, código e resultados.

## Escopo

| O que fazemos | O que NÃO fazemos |
|---------------|-------------------|
| Modelagem física 1D | Otimização de desempenho |
| Simulação de sensores | Tempo real |
| Filtro de Kalman 1D | Hardware embarcado |
| Fusão INS/GPS | Movimento 3D (nesta fase) |
| Análise de deriva | Notebooks Jupyter |

## Estrutura do Projeto

```
ins-navigation-project/
├── src/
│   ├── models/          # Modelo físico do movimento
│   ├── sensors/         # Simuladores de sensores
│   ├── filters/         # Filtros de estimação
│   └── utils/           # Funções utilitárias
├── experiments/         # Scripts de simulação
├── docs/                # Documentação
└── results/             # Resultados gerados
```

## Fluxo de Dados

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Trajetória     │────▶│   Acelerômetro  │────▶│   Integração    │
│  Verdadeira     │     │   (bias+ruído)  │     │   (INS puro)    │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
        │                                                 │
        │               ┌─────────────────┐              │
        └──────────────▶│      GPS        │──────────────┤
                        │    (ruído)      │              │
                        └─────────────────┘              ▼
                                               ┌─────────────────┐
                                               │  Filtro de      │
                                               │  Kalman (fusão) │
                                               └─────────────────┘
```

## Status Atual

- [x] Estrutura de diretórios
- [x] Modelo físico 1D
- [x] Simulador de acelerômetro
- [x] Simulador de GPS
- [x] Filtro de Kalman 1D
- [x] Experimento principal
- [x] Documentação base

## Próximos Passos (Expansões Futuras)

1. Implementar filtro EKF para modelos não-lineares
2. Expandir para 2D/3D
3. Adicionar modelo de viés variável (random walk)
4. Portar para C++ para validação de desempenho

## Referências

- Titterton & Weston, "Strapdown Inertial Navigation Technology"
- Welch & Bishop, "An Introduction to the Kalman Filter"
- Kaplan & Hegarty, "Understanding GPS: Principles and Applications"
