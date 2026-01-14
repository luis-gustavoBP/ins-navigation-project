# Sistema de Navegação Inercial 1D

**Projeto acadêmico de fusão INS/GPS utilizando Filtro de Kalman**

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## 📋 Visão Geral

Este projeto implementa um Sistema de Navegação Inercial (INS) em 1 dimensão para fins de estudo acadêmico. O foco é a **clareza conceitual** e **rastreabilidade matemática**, não otimização de desempenho.

### Objetivos

- ✅ Demonstrar a deriva do INS puro (sem GPS)
- ✅ Mostrar a correção pela fusão INS/GPS
- ✅ Permitir ajuste consciente de Q e R
- ✅ Servir como base para expansão a 3D e C++

## 🏗️ Estrutura do Projeto

```
ins-navigation-project/
│
├── src/                          # Código fonte
│   ├── models/
│   │   └── motion_1d.py          # Modelo físico 1D
│   ├── sensors/
│   │   ├── accelerometer_1d.py   # Simulador de acelerômetro
│   │   └── gps_1d.py             # Simulador de GPS
│   ├── filters/
│   │   └── kalman_1d.py          # Filtro de Kalman 1D
│   └── utils/
│       └── noise.py              # Funções de ruído
│
├── experiments/                  # Scripts de experimento
│   └── exp_01_kalman_1d.py       # Experimento principal
│
├── docs/                         # Documentação
│   ├── 00_visao_geral.md
│   ├── 01_modelo_fisico.md
│   ├── 02_modelo_estatistico.md
│   ├── 03_kalman_1d.md
│   └── 04_resultados.md
│
├── results/                      # Resultados gerados
└── README.md
```

## 🚀 Início Rápido

### Pré-requisitos

```bash
pip install numpy matplotlib
```

### Execução

```bash
cd ins-navigation-project
python experiments/exp_01_kalman_1d.py
```

### Resultado Esperado

O script irá:
1. Simular uma trajetória 1D com aceleração variável
2. Executar navegação inercial pura (INS)
3. Executar fusão INS/GPS com Filtro de Kalman
4. Gerar gráficos comparativos em `results/`

## 📖 Documentação

| Documento | Conteúdo |
|-----------|----------|
| [00_visao_geral.md](docs/00_visao_geral.md) | Escopo e status do projeto |
| [01_modelo_fisico.md](docs/01_modelo_fisico.md) | Equações do movimento 1D |
| [02_modelo_estatistico.md](docs/02_modelo_estatistico.md) | Ruídos, Q e R |
| [03_kalman_1d.md](docs/03_kalman_1d.md) | Derivação do Filtro de Kalman |
| [04_resultados.md](docs/04_resultados.md) | Análise dos resultados |

## 🔬 Conceitos Principais

### Modelo Físico

```
x[k+1] = x[k] + v[k]·dt + ½·a[k]·dt²
v[k+1] = v[k] + a[k]·dt
```

### Modelo de Sensores

- **Acelerômetro**: `a_med = a_true + bias + ruído`
- **GPS**: `z_gps = x_true + ruído`

### Filtro de Kalman

```
PREDIÇÃO:  x⁻ = F·x + B·a,  P⁻ = F·P·Fᵀ + Q
CORREÇÃO:  K = P⁻·Hᵀ·(H·P⁻·Hᵀ + R)⁻¹
           x = x⁻ + K·(z - H·x⁻)
           P = (I - K·H)·P⁻
```

## ⚙️ Parâmetros Configuráveis

No arquivo `experiments/exp_01_kalman_1d.py`:

```python
# Sensores
ACC_BIAS = 0.02       # Viés do acelerômetro [m/s²]
ACC_NOISE_STD = 0.1   # Ruído do acelerômetro [m/s²]
GPS_NOISE_STD = 3.0   # Ruído do GPS [m]

# Filtro de Kalman
PROCESS_NOISE_STD = 0.5  # Afeta matriz Q
```

## 📊 Resultados Típicos

| Métrica | INS Puro | Kalman |
|---------|----------|--------|
| Erro RMS | ~60 m | < 4 m |
| Deriva | Quadrática | Limitada |

## 🎯 Próximos Passos

- [ ] Expansão para 2D/3D
- [ ] Viés variável (random walk)
- [ ] Comparação EKF vs UKF
- [ ] Portabilidade para C++

## 📚 Referências

- Titterton & Weston, "Strapdown Inertial Navigation Technology"
- Welch & Bishop, "An Introduction to the Kalman Filter"
- Kaplan & Hegarty, "Understanding GPS: Principles and Applications"

## 📄 Licença

Este projeto é para fins acadêmicos (IC/TCC).
