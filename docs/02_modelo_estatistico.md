# Modelo Estatístico - Ruídos e Incertezas

## Introdução

Este documento descreve o modelo estatístico utilizado para representar ruídos e incertezas no sistema INS/GPS. A correta modelagem estatística é fundamental para a sintonia do Filtro de Kalman.

## Tipos de Erro

### 1. Ruído Branco Gaussiano

Erro aleatório sem correlação temporal:

$$
w[k] \sim \mathcal{N}(0, \sigma^2)
$$

**Propriedades:**
- Média zero: $E[w] = 0$
- Variância constante: $E[w^2] = \sigma^2$
- Descorrelacionado: $E[w[k] \cdot w[j]] = 0$ para $k \neq j$

### 2. Viés (Bias)

Erro sistemático constante:

$$
b = \text{constante}
$$

**Efeito no INS:**
- Viés de aceleração causa deriva quadrática na posição
- $\Delta x(t) \approx \frac{1}{2} b \cdot t^2$

## Modelo do Acelerômetro

A medição do acelerômetro é modelada como:

$$
a_{med} = a_{true} + b + w_a
$$

Onde:
- $a_{true}$ = aceleração verdadeira
- $b$ = viés do sensor (constante)
- $w_a \sim \mathcal{N}(0, \sigma_a^2)$ = ruído branco

### Parâmetros Típicos (MEMS)

| Parâmetro | Símbolo | Valor Típico | Unidade |
|-----------|---------|--------------|---------|
| Viés | $b$ | 0.01 - 0.1 | m/s² |
| Ruído | $\sigma_a$ | 0.01 - 0.1 | m/s² |

## Modelo do GPS

A medição de posição do GPS é modelada como:

$$
z_{gps} = x_{true} + w_{gps}
$$

Onde:
- $x_{true}$ = posição verdadeira
- $w_{gps} \sim \mathcal{N}(0, \sigma_{gps}^2)$ = ruído de medição

### Parâmetros Típicos

| Tipo de GPS | $\sigma_{gps}$ |
|-------------|----------------|
| Civil | 3 - 5 m |
| DGPS | 0.5 - 2 m |
| RTK | 0.01 - 0.02 m |

## Matrizes de Covariância

### Matriz Q - Ruído de Processo

Modela a incerteza no modelo dinâmico. Para ruído de aceleração:

$$
\mathbf{Q} = \mathbf{G} \cdot \sigma_a^2 \cdot \mathbf{G}^T
$$

Onde $\mathbf{G} = \mathbf{B}$ (matriz de controle):

$$
\mathbf{Q} = \begin{bmatrix} 
\frac{\Delta t^4}{4} & \frac{\Delta t^3}{2} \\ 
\frac{\Delta t^3}{2} & \Delta t^2 
\end{bmatrix} \sigma_a^2
$$

**Interpretação:**
- Q grande → modelo pouco confiável → filtro "acredita mais" nas medições
- Q pequeno → modelo muito confiável → filtro "acredita mais" na predição

### Matriz R - Ruído de Medição

Modela a incerteza nas medições do GPS:

$$
\mathbf{R} = \begin{bmatrix} \sigma_{gps}^2 \end{bmatrix}
$$

**Interpretação:**
- R grande → medição ruidosa → filtro suaviza mais
- R pequeno → medição confiável → filtro segue medição

## Sintonia Q vs R

A razão Q/R determina o comportamento do filtro:

| Razão Q/R | Comportamento |
|-----------|---------------|
| Alto | Filtro responsivo, mas ruidoso |
| Baixo | Filtro suave, mas lento |
| Equilibrado | Compromisso ótimo |

```
        ┌──────────────────────────────────────────────────┐
        │                                                  │
Alto Q  │  ●●●●    Ruidoso, responsivo                    │
        │     ●●●●                                         │
        │         ●●●●                                     │
        │                                                  │
        │  ────────────────────── (referência)            │
        │                                                  │
Baixo Q │  ─────────────────────────────                  │
        │         Suave, mas com atraso                   │
        │                                                  │
        └──────────────────────────────────────────────────┘
                              Tempo →
```

## Hipóteses do Modelo

1. **Ruído gaussiano**: distribuição normal para todos os ruídos
2. **Ruído branco**: sem correlação temporal
3. **Viés constante**: não modela deriva do viés (random walk)
4. **Independência**: ruídos de diferentes sensores são independentes
5. **Conhecimento de σ**: parâmetros estatísticos são conhecidos

## Implementação

```python
# Ruído branco
w = np.random.normal(0, sigma)

# Medição corrompida
measurement = true_value + bias + w

# Matriz Q
Q = np.array([
    [dt**4/4, dt**3/2],
    [dt**3/2, dt**2]
]) * sigma_a**2

# Matriz R
R = np.array([[sigma_gps**2]])
```
