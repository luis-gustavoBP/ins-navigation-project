# Resultados - Experimento 01

## Configuração do Experimento

| Parâmetro | Valor | Unidade |
|-----------|-------|---------|
| Tempo total | 100 | s |
| Intervalo (dt) | 0.1 | s |
| Taxa GPS | 1 | Hz |
| Viés acelerômetro | 0.02 | m/s² |
| Ruído acelerômetro | 0.1 | m/s² |
| Ruído GPS | 3.0 | m |

## Gráficos

> Execute o experimento para gerar os gráficos:
> ```bash
> python experiments/exp_01_kalman_1d.py
> ```
> Os resultados serão salvos em `results/exp_01_results.png`

## Análise Esperada

### 1. Deriva do INS Puro

O INS puro (sem correção GPS) apresenta **deriva quadrática** devido ao viés do acelerômetro:

$$
\Delta x(t) \approx \frac{1}{2} b_{acc} \cdot t^2
$$

Para $b = 0.02$ m/s² e $t = 100$ s:
$$
\Delta x \approx \frac{1}{2} \times 0.02 \times 100^2 = 100 \text{ m}
$$

**Interpretação física:** O viés é integrado duas vezes (para velocidade e depois para posição), causando erro que cresce com $t^2$.

### 2. Correção pela Fusão INS/GPS

O Filtro de Kalman limita a deriva ao combinar:
- **Predição INS**: alta taxa, curto prazo, mas com deriva
- **Correção GPS**: baixa taxa, sem deriva, mas ruidoso

O erro de posição oscila em torno de zero com amplitude proporcional ao ruído do GPS.

### 3. Comportamento da Incerteza

A covariância $P$ do Kalman:
- **Cresce** durante predição (sem GPS)
- **Diminui** após atualização com GPS
- **Oscila** em regime permanente (padrão dente-de-serra)

## Métricas de Desempenho

| Métrica | INS Puro | Kalman (INS+GPS) |
|---------|----------|------------------|
| Erro médio | ~50 m | < 5 m |
| Erro RMS | ~60 m | < 4 m |
| Erro máximo | ~100 m | < 10 m |

> **Nota:** Valores ilustrativos. Execute o experimento para valores exatos.

## Interpretação Física

### Por que o INS deriva?

1. Viés de aceleração → erro de velocidade cresce linearmente
2. Erro de velocidade → erro de posição cresce quadraticamente
3. Sem referência externa → não há como corrigir

### Por que a fusão funciona?

1. GPS fornece referência absoluta de posição
2. Kalman pondera INS (curto prazo) vs GPS (longo prazo)
3. Correções periódicas "resetam" a deriva

## Experimentos Adicionais Sugeridos

### Variar Q (ruído de processo)
- Q alto: filtro mais responsivo, mais ruidoso
- Q baixo: filtro mais suave, mais lento

### Variar R (ruído de medição)
- R alto: GPS menos confiável, mais peso para INS
- R baixo: GPS mais confiável, segue GPS de perto

### Variar taxa do GPS
- 10 Hz: correções frequentes, erro menor
- 0.1 Hz: correções raras, maior deriva entre correções

## Conclusões

1. **INS puro é inadequado** para navegação de longo prazo devido à deriva
2. **Fusão INS/GPS** combina os pontos fortes de cada sensor
3. **Sintonia Q/R** permite ajustar o comportamento do filtro
4. O modelo 1D captura a essência do problema e serve de base para extensões
