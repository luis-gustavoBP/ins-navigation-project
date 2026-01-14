# Modelo Físico - Movimento 1D

## Descrição

Este documento descreve o modelo físico utilizado para representar o movimento de uma partícula em uma dimensão (1D). O modelo é baseado nas equações da cinemática clássica.

## Equações Contínuas

O movimento em 1D é descrito pelo sistema de equações diferenciais:

$$
\frac{dx}{dt} = v(t)
$$

$$
\frac{dv}{dt} = a(t)
$$

Onde:
- $x(t)$ = posição [m]
- $v(t)$ = velocidade [m/s]
- $a(t)$ = aceleração [m/s²]

## Representação em Espaço de Estados

O sistema pode ser escrito na forma de espaço de estados:

$$
\mathbf{x} = \begin{bmatrix} x \\ v \end{bmatrix}
$$

$$
\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \mathbf{x} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} a
$$

## Discretização

Para implementação computacional, discretizamos as equações usando o método de Euler com intervalo $\Delta t$:

### Equações Discretizadas

$$
x[k+1] = x[k] + v[k] \cdot \Delta t + \frac{1}{2} a[k] \cdot \Delta t^2
$$

$$
v[k+1] = v[k] + a[k] \cdot \Delta t
$$

### Forma Matricial

$$
\mathbf{x}[k+1] = \mathbf{F} \cdot \mathbf{x}[k] + \mathbf{B} \cdot u[k]
$$

Onde:

**Matriz de Transição de Estado:**
$$
\mathbf{F} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

**Matriz de Controle:**
$$
\mathbf{B} = \begin{bmatrix} \frac{\Delta t^2}{2} \\ \Delta t \end{bmatrix}
$$

**Entrada de Controle:**
$$
u[k] = a[k]
$$

## Implementação

O modelo físico está implementado em `src/models/motion_1d.py`:

```python
# Matriz de transição
F = np.array([
    [1.0, dt],
    [0.0, 1.0]
])

# Matriz de controle
B = np.array([
    [0.5 * dt**2],
    [dt]
])

# Propagação do estado
x_new = F @ x + B @ np.array([[acceleration]])
```

## Hipóteses do Modelo

1. **Movimento retilíneo**: apenas uma dimensão espacial
2. **Massa pontual**: sem rotação ou dimensões físicas
3. **Aceleração conhecida**: a(t) é uma entrada de controle
4. **Sem atrito**: modelo ideal, sem forças dissipativas
5. **Discretização adequada**: dt pequeno o suficiente para Euler

## Diagrama do Sistema

```
                    ┌─────────────────────────────────┐
   a[k] ──────────▶ │           Modelo 1D             │
   (aceleração)     │                                 │
                    │  x[k+1] = F·x[k] + B·a[k]       │
                    │                                 │
                    │  Estado: [posição, velocidade]  │
                    └─────────────────────────────────┘
                                    │
                                    ▼
                              [x[k+1], v[k+1]]
```

## Limitações

- Modelo linear (não captura efeitos não-lineares)
- Discretização por Euler (erro O(Δt²) por passo)
- Não modela ruídos ou incertezas (modelo determinístico)
