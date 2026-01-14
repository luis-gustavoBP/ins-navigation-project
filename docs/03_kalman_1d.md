# Filtro de Kalman 1D - Derivação Matemática

## Introdução

O Filtro de Kalman é um estimador ótimo recursivo que combina informações de um modelo dinâmico com medições ruidosas para produzir estimativas de mínima variância.

## Formulação do Problema

### Estado

$$
\mathbf{x} = \begin{bmatrix} x \\ v \end{bmatrix}
$$

Onde:
- $x$ = posição [m]
- $v$ = velocidade [m/s]

### Modelo de Processo

$$
\mathbf{x}[k] = \mathbf{F} \mathbf{x}[k-1] + \mathbf{B} u[k-1] + \mathbf{w}[k-1]
$$

Onde:
- $\mathbf{F}$ = matriz de transição de estado
- $\mathbf{B}$ = matriz de controle
- $u$ = entrada de controle (aceleração)
- $\mathbf{w} \sim \mathcal{N}(0, \mathbf{Q})$ = ruído de processo

### Modelo de Medição

$$
\mathbf{z}[k] = \mathbf{H} \mathbf{x}[k] + \mathbf{v}[k]
$$

Onde:
- $\mathbf{H}$ = matriz de observação
- $\mathbf{v} \sim \mathcal{N}(0, \mathbf{R})$ = ruído de medição

## Matrizes do Sistema

### Transição de Estado

$$
\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

### Controle

$$
\mathbf{B} = \begin{bmatrix} \frac{\Delta t^2}{2} \\ \Delta t \end{bmatrix}
$$

### Observação

O GPS mede apenas posição:

$$
\mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

## Algoritmo do Filtro de Kalman

O filtro opera em dois passos: **Predição** e **Correção**.

```
╔══════════════════════════════════════════════════════════════╗
║                    ALGORITMO COMPLETO                        ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  ┌─────────────── PREDIÇÃO ────────────────────────────────┐ ║
║  │                                                          │ ║
║  │  (1) Estado a priori:                                   │ ║
║  │      x̂⁻[k] = F·x̂[k-1] + B·u[k-1]                        │ ║
║  │                                                          │ ║
║  │  (2) Covariância a priori:                              │ ║
║  │      P⁻[k] = F·P[k-1]·Fᵀ + Q                            │ ║
║  │                                                          │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                           │                                  ║
║                           ▼                                  ║
║  ┌─────────────── CORREÇÃO ────────────────────────────────┐ ║
║  │                                                          │ ║
║  │  (3) Ganho de Kalman:                                   │ ║
║  │      K[k] = P⁻[k]·Hᵀ·(H·P⁻[k]·Hᵀ + R)⁻¹                │ ║
║  │                                                          │ ║
║  │  (4) Estado a posteriori:                                │ ║
║  │      x̂[k] = x̂⁻[k] + K[k]·(z[k] - H·x̂⁻[k])              │ ║
║  │                                                          │ ║
║  │  (5) Covariância a posteriori:                          │ ║
║  │      P[k] = (I - K[k]·H)·P⁻[k]                          │ ║
║  │                                                          │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
```

## Derivação Detalhada

### Passo 1: Predição do Estado

Usando o modelo dinâmico para propagar o estado:

$$
\hat{\mathbf{x}}^-[k] = \mathbf{F} \hat{\mathbf{x}}[k-1] + \mathbf{B} u[k-1]
$$

O superíndice $^-$ indica estimativa a priori (antes da medição).

### Passo 2: Predição da Covariância

A incerteza também é propagada:

$$
\mathbf{P}^-[k] = \mathbf{F} \mathbf{P}[k-1] \mathbf{F}^T + \mathbf{Q}
$$

### Passo 3: Ganho de Kalman

O ganho determina o peso entre predição e medição:

$$
\mathbf{K}[k] = \mathbf{P}^-[k] \mathbf{H}^T (\mathbf{H} \mathbf{P}^-[k] \mathbf{H}^T + \mathbf{R})^{-1}
$$

**Interpretação do Ganho:**
- $\mathbf{K} \to 0$: ignora medição, confia na predição
- $\mathbf{K} \to 1$: ignora predição, segue medição

### Passo 4: Atualização do Estado

Corrige a predição usando a inovação (resíduo):

$$
\hat{\mathbf{x}}[k] = \hat{\mathbf{x}}^-[k] + \mathbf{K}[k] (\mathbf{z}[k] - \mathbf{H} \hat{\mathbf{x}}^-[k])
$$

O termo $\mathbf{z}[k] - \mathbf{H} \hat{\mathbf{x}}^-[k]$ é a **inovação**.

### Passo 5: Atualização da Covariância

$$
\mathbf{P}[k] = (\mathbf{I} - \mathbf{K}[k] \mathbf{H}) \mathbf{P}^-[k]
$$

## Exemplo Numérico

Para $\Delta t = 0.1$ s:

$$
\mathbf{F} = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}, \quad
\mathbf{B} = \begin{bmatrix} 0.005 \\ 0.1 \end{bmatrix}
$$

$$
\mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

## Propriedades do Filtro

1. **Optimalidade**: minimiza o erro quadrático médio
2. **Recursividade**: não precisa de todo o histórico
3. **Gaussianidade**: preserva distribuição normal
4. **Estabilidade**: converge sob observabilidade

## Convergência

A covariância converge para um valor estacionário $\mathbf{P}_\infty$ quando:
- Sistema é observável
- Sistema é controlável
- Parâmetros Q e R são consistentes

## Implementação

```python
# Predição
x_pred = F @ x + B @ u
P_pred = F @ P @ F.T + Q

# Inovação
S = H @ P_pred @ H.T + R
K = P_pred @ H.T @ np.linalg.inv(S)

# Correção
y = z - H @ x_pred  # inovação
x = x_pred + K @ y
P = (I - K @ H) @ P_pred
```

## Referências

- Welch, G. & Bishop, G. "An Introduction to the Kalman Filter" (2006)
- Bar-Shalom, Y. "Estimation with Applications to Tracking and Navigation" (2001)
