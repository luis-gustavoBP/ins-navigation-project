#!/usr/bin/env python3
"""
Experimento 01: Filtro de Kalman 1D - Fusão INS/GPS

Este experimento demonstra:
    1. Deriva do INS puro (sem correção GPS)
    2. Correção introduzida pela fusão INS/GPS
    3. Efeito da sintonia das matrizes Q e R

Cenário:
    - Movimento 1D com aceleração variável
    - Acelerômetro com viés e ruído
    - GPS com ruído alto (mas sem viés)
    - Taxa do GPS menor que a do acelerômetro

Saídas:
    - Gráficos comparativos
    - Análise de erro

Autor: Projeto INS 1D
Data: 2024
"""

import sys
from pathlib import Path

# Adiciona o diretório src ao path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Importações dos módulos do projeto
from models.motion_1d import MotionModel1D, State1D
from sensors.accelerometer_1d import Accelerometer1D, AccelerometerParams
from sensors.gps_1d import GPS1D, GPSParams
from filters.kalman_1d import KalmanFilter1D
from utils.noise import set_seed


# =============================================================================
# CONFIGURAÇÃO DO EXPERIMENTO
# =============================================================================

# Reprodutibilidade
SEED = 42

# Parâmetros temporais
DT = 0.1              # Intervalo de amostragem [s]
T_TOTAL = 100.0       # Tempo total de simulação [s]
GPS_RATE = 1.0        # Taxa de amostragem do GPS [Hz]

# Parâmetros do sensor - Acelerômetro
ACC_BIAS = 0.02       # Viés do acelerômetro [m/s²]
ACC_NOISE_STD = 0.1   # Ruído do acelerômetro [m/s²]

# Parâmetros do sensor - GPS
GPS_NOISE_STD = 3.0   # Ruído do GPS [m]

# Parâmetros do Filtro de Kalman
# Q: Ruído de processo (incerteza no modelo)
PROCESS_NOISE_STD = 0.5

# NOTA: R é derivado de GPS_NOISE_STD


# =============================================================================
# FUNÇÕES AUXILIARES
# =============================================================================

def generate_acceleration_profile(n_steps: int, dt: float) -> np.ndarray:
    """
    Gera um perfil de aceleração para o experimento.
    
    Perfil: combinação de acelerações constantes e senoidais
    para criar movimento interessante.
    
    Args:
        n_steps: Número de passos de tempo.
        dt: Intervalo de tempo.
        
    Returns:
        Array de acelerações [m/s²].
    """
    t = np.arange(n_steps) * dt
    
    # Perfil de aceleração:
    # - Aceleração inicial positiva
    # - Oscilação senoidal
    # - Desaceleração final
    
    acc = np.zeros(n_steps)
    
    # Fase 1: Aceleração (0-20s)
    mask1 = t < 20
    acc[mask1] = 0.5
    
    # Fase 2: Cruzeiro com oscilações (20-60s)
    mask2 = (t >= 20) & (t < 60)
    acc[mask2] = 0.1 * np.sin(0.5 * t[mask2])
    
    # Fase 3: Desaceleração (60-80s)
    mask3 = (t >= 60) & (t < 80)
    acc[mask3] = -0.3
    
    # Fase 4: Parado (80-100s)
    mask4 = t >= 80
    acc[mask4] = 0.0
    
    return acc


def run_ins_only(
    model: MotionModel1D,
    accelerometer: Accelerometer1D,
    true_accelerations: np.ndarray,
    initial_state: State1D
) -> tuple:
    """
    Executa navegação inercial pura (sem GPS).
    
    Demonstra a deriva causada pelo viés do acelerômetro.
    
    Args:
        model: Modelo de movimento.
        accelerometer: Sensor de aceleração.
        true_accelerations: Acelerações verdadeiras.
        initial_state: Estado inicial.
        
    Returns:
        Tupla (posições_estimadas, velocidades_estimadas).
    """
    n_steps = len(true_accelerations)
    
    positions = np.zeros(n_steps + 1)
    velocities = np.zeros(n_steps + 1)
    
    positions[0] = initial_state.position
    velocities[0] = initial_state.velocity
    
    state = initial_state
    
    for k, true_acc in enumerate(true_accelerations):
        # Medição do acelerômetro (com viés e ruído)
        measured_acc = accelerometer.measure(true_acc)
        
        # Integração usando aceleração medida
        state = model.predict(state, measured_acc)
        
        positions[k + 1] = state.position
        velocities[k + 1] = state.velocity
    
    return positions, velocities


def run_kalman_fusion(
    accelerometer: Accelerometer1D,
    gps: GPS1D,
    true_accelerations: np.ndarray,
    true_positions: np.ndarray,
    kalman_filter: KalmanFilter1D,
    gps_interval: int
) -> tuple:
    """
    Executa fusão INS/GPS com Filtro de Kalman.
    
    Args:
        accelerometer: Sensor de aceleração.
        gps: Sensor GPS.
        true_accelerations: Acelerações verdadeiras.
        true_positions: Posições verdadeiras.
        kalman_filter: Filtro de Kalman configurado.
        gps_interval: Intervalo entre medições GPS (em passos).
        
    Returns:
        Tupla (posições_estimadas, velocidades_estimadas, 
               covariâncias_posição, medições_gps).
    """
    n_steps = len(true_accelerations)
    
    positions = np.zeros(n_steps + 1)
    velocities = np.zeros(n_steps + 1)
    covariances = np.zeros(n_steps + 1)
    gps_measurements = []
    gps_times = []
    
    # Estado inicial
    state = kalman_filter.state
    positions[0] = state.position
    velocities[0] = state.velocity
    covariances[0] = state.position_variance
    
    for k in range(n_steps):
        # Medição do acelerômetro
        measured_acc = accelerometer.measure(true_accelerations[k])
        
        # Predição
        kalman_filter.predict(measured_acc)
        
        # Correção (se GPS disponível)
        if (k + 1) % gps_interval == 0:
            gps_pos = gps.measure(true_positions[k + 1])
            kalman_filter.update(gps_pos)
            gps_measurements.append(gps_pos)
            gps_times.append(k + 1)
        
        # Armazena resultados
        state = kalman_filter.state
        positions[k + 1] = state.position
        velocities[k + 1] = state.velocity
        covariances[k + 1] = state.position_variance
    
    return positions, velocities, covariances, (gps_times, gps_measurements)


def plot_results(
    times: np.ndarray,
    true_pos: np.ndarray,
    true_vel: np.ndarray,
    ins_pos: np.ndarray,
    ins_vel: np.ndarray,
    kf_pos: np.ndarray,
    kf_vel: np.ndarray,
    kf_cov: np.ndarray,
    gps_data: tuple,
    save_path: Path = None
) -> None:
    """
    Gera gráficos comparativos dos resultados.
    
    Args:
        times: Array de tempos.
        true_pos: Posições verdadeiras.
        true_vel: Velocidades verdadeiras.
        ins_pos: Posições estimadas pelo INS puro.
        ins_vel: Velocidades estimadas pelo INS puro.
        kf_pos: Posições estimadas pelo Kalman.
        kf_vel: Velocidades estimadas pelo Kalman.
        kf_cov: Covariância de posição do Kalman.
        gps_data: Tupla (tempos_gps, medições_gps).
        save_path: Caminho para salvar a figura.
    """
    gps_times, gps_meas = gps_data
    gps_t = times[gps_times] if gps_times else []
    
    fig = plt.figure(figsize=(14, 10))
    gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.25)
    
    # =========================================================================
    # Gráfico 1: Posição
    # =========================================================================
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(times, true_pos, 'k-', linewidth=2, label='Verdadeiro')
    ax1.plot(times, ins_pos, 'r--', linewidth=1.5, alpha=0.8, label='INS Puro')
    ax1.plot(times, kf_pos, 'b-', linewidth=1.5, label='Kalman (INS+GPS)')
    if len(gps_meas) > 0:
        ax1.scatter(gps_t, gps_meas, c='green', s=20, alpha=0.5, 
                   label='Medições GPS', zorder=5)
    ax1.set_xlabel('Tempo [s]')
    ax1.set_ylabel('Posição [m]')
    ax1.set_title('Comparação de Posição')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    
    # =========================================================================
    # Gráfico 2: Velocidade
    # =========================================================================
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(times, true_vel, 'k-', linewidth=2, label='Verdadeiro')
    ax2.plot(times, ins_vel, 'r--', linewidth=1.5, alpha=0.8, label='INS Puro')
    ax2.plot(times, kf_vel, 'b-', linewidth=1.5, label='Kalman (INS+GPS)')
    ax2.set_xlabel('Tempo [s]')
    ax2.set_ylabel('Velocidade [m/s]')
    ax2.set_title('Comparação de Velocidade')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    # =========================================================================
    # Gráfico 3: Erro de Posição
    # =========================================================================
    ax3 = fig.add_subplot(gs[1, 0])
    error_ins = ins_pos - true_pos
    error_kf = kf_pos - true_pos
    ax3.plot(times, error_ins, 'r-', linewidth=1.5, alpha=0.8, label='INS Puro')
    ax3.plot(times, error_kf, 'b-', linewidth=1.5, label='Kalman')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.fill_between(times, -np.sqrt(kf_cov), np.sqrt(kf_cov), 
                     alpha=0.2, color='blue', label='±1σ Kalman')
    ax3.set_xlabel('Tempo [s]')
    ax3.set_ylabel('Erro de Posição [m]')
    ax3.set_title('Erro de Posição ao Longo do Tempo')
    ax3.legend(loc='upper left')
    ax3.grid(True, alpha=0.3)
    
    # =========================================================================
    # Gráfico 4: Erro de Velocidade
    # =========================================================================
    ax4 = fig.add_subplot(gs[1, 1])
    error_vel_ins = ins_vel - true_vel
    error_vel_kf = kf_vel - true_vel
    ax4.plot(times, error_vel_ins, 'r-', linewidth=1.5, alpha=0.8, label='INS Puro')
    ax4.plot(times, error_vel_kf, 'b-', linewidth=1.5, label='Kalman')
    ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Tempo [s]')
    ax4.set_ylabel('Erro de Velocidade [m/s]')
    ax4.set_title('Erro de Velocidade ao Longo do Tempo')
    ax4.legend(loc='upper left')
    ax4.grid(True, alpha=0.3)
    
    # =========================================================================
    # Gráfico 5: Deriva do INS (análise)
    # =========================================================================
    ax5 = fig.add_subplot(gs[2, 0])
    # Erro quadrático acumulado (demonstra deriva)
    ax5.semilogy(times, np.abs(error_ins) + 1e-6, 'r-', linewidth=1.5, 
                 label='|Erro INS|')
    ax5.semilogy(times, np.abs(error_kf) + 1e-6, 'b-', linewidth=1.5, 
                 label='|Erro Kalman|')
    ax5.set_xlabel('Tempo [s]')
    ax5.set_ylabel('|Erro de Posição| [m] (log)')
    ax5.set_title('Deriva: INS Puro vs Kalman (escala log)')
    ax5.legend(loc='upper left')
    ax5.grid(True, alpha=0.3, which='both')
    
    # =========================================================================
    # Gráfico 6: Incerteza do Kalman
    # =========================================================================
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(times, np.sqrt(kf_cov), 'b-', linewidth=1.5)
    ax6.set_xlabel('Tempo [s]')
    ax6.set_ylabel('σ posição [m]')
    ax6.set_title('Incerteza de Posição do Filtro de Kalman')
    ax6.grid(True, alpha=0.3)
    
    plt.suptitle('Experimento 01: Fusão INS/GPS com Filtro de Kalman 1D', 
                 fontsize=14, fontweight='bold')
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Figura salva em: {save_path}")
    
    plt.show()


def print_statistics(
    times: np.ndarray,
    true_pos: np.ndarray,
    ins_pos: np.ndarray,
    kf_pos: np.ndarray
) -> None:
    """
    Imprime estatísticas de erro.
    """
    error_ins = ins_pos - true_pos
    error_kf = kf_pos - true_pos
    
    print("\n" + "=" * 60)
    print("ESTATÍSTICAS DE ERRO")
    print("=" * 60)
    
    print(f"\n{'Métrica':<30} {'INS Puro':>12} {'Kalman':>12}")
    print("-" * 60)
    
    print(f"{'Erro médio [m]':<30} {np.mean(error_ins):>12.3f} {np.mean(error_kf):>12.3f}")
    print(f"{'Erro RMS [m]':<30} {np.sqrt(np.mean(error_ins**2)):>12.3f} {np.sqrt(np.mean(error_kf**2)):>12.3f}")
    print(f"{'Erro máximo [m]':<30} {np.max(np.abs(error_ins)):>12.3f} {np.max(np.abs(error_kf)):>12.3f}")
    print(f"{'Erro final [m]':<30} {error_ins[-1]:>12.3f} {error_kf[-1]:>12.3f}")
    
    print("\n" + "=" * 60)


# =============================================================================
# EXECUÇÃO PRINCIPAL
# =============================================================================

def main():
    """Função principal do experimento."""
    
    print("=" * 60)
    print("EXPERIMENTO 01: FILTRO DE KALMAN 1D - FUSÃO INS/GPS")
    print("=" * 60)
    
    # -------------------------------------------------------------------------
    # Configuração
    # -------------------------------------------------------------------------
    set_seed(SEED)
    
    n_steps = int(T_TOTAL / DT)
    gps_interval = int(1.0 / (GPS_RATE * DT))  # Passos entre medições GPS
    
    print(f"\nParâmetros da Simulação:")
    print(f"  Tempo total: {T_TOTAL} s")
    print(f"  Intervalo: {DT} s")
    print(f"  Passos: {n_steps}")
    print(f"  Intervalo GPS: {gps_interval} passos ({1/GPS_RATE:.1f}s)")
    
    # -------------------------------------------------------------------------
    # Inicialização
    # -------------------------------------------------------------------------
    
    # Modelo físico
    model = MotionModel1D(dt=DT)
    
    # Sensores
    accelerometer = Accelerometer1D(
        AccelerometerParams(bias=ACC_BIAS, noise_std=ACC_NOISE_STD)
    )
    gps = GPS1D(GPSParams(noise_std=GPS_NOISE_STD))
    
    print(f"\n{accelerometer.describe()}")
    print(f"\n{gps.describe()}")
    
    # Estado inicial
    initial_state = State1D(position=0.0, velocity=0.0)
    
    # Filtro de Kalman
    kalman = KalmanFilter1D(
        dt=DT,
        process_noise_std=PROCESS_NOISE_STD,
        measurement_noise_std=GPS_NOISE_STD,
        initial_position=0.0,
        initial_velocity=0.0,
        initial_covariance=1.0
    )
    
    # -------------------------------------------------------------------------
    # Geração da Trajetória Verdadeira
    # -------------------------------------------------------------------------
    
    print("\nGerando trajetória verdadeira...")
    true_accelerations = generate_acceleration_profile(n_steps, DT)
    times, true_positions, true_velocities = model.simulate_trajectory(
        initial_state, true_accelerations
    )
    
    print(f"  Posição final verdadeira: {true_positions[-1]:.2f} m")
    print(f"  Velocidade final verdadeira: {true_velocities[-1]:.2f} m/s")
    
    # -------------------------------------------------------------------------
    # Simulação INS Puro
    # -------------------------------------------------------------------------
    
    print("\nExecutando INS puro (sem GPS)...")
    set_seed(SEED)  # Reset para mesmas realizações de ruído
    ins_positions, ins_velocities = run_ins_only(
        model, accelerometer, true_accelerations, initial_state
    )
    
    print(f"  Posição final INS: {ins_positions[-1]:.2f} m")
    print(f"  Erro de posição: {ins_positions[-1] - true_positions[-1]:.2f} m")
    
    # -------------------------------------------------------------------------
    # Simulação Kalman (INS + GPS)
    # -------------------------------------------------------------------------
    
    print("\nExecutando Filtro de Kalman (INS + GPS)...")
    set_seed(SEED)  # Reset para mesmas realizações de ruído
    kf_positions, kf_velocities, kf_covariances, gps_data = run_kalman_fusion(
        accelerometer, gps, true_accelerations, true_positions,
        kalman, gps_interval
    )
    
    print(f"  Posição final Kalman: {kf_positions[-1]:.2f} m")
    print(f"  Erro de posição: {kf_positions[-1] - true_positions[-1]:.2f} m")
    print(f"  Incerteza (1σ): {np.sqrt(kf_covariances[-1]):.2f} m")
    
    # -------------------------------------------------------------------------
    # Resultados
    # -------------------------------------------------------------------------
    
    print_statistics(times, true_positions, ins_positions, kf_positions)
    
    # -------------------------------------------------------------------------
    # Visualização
    # -------------------------------------------------------------------------
    
    print("\nGerando gráficos...")
    
    # Diretório para salvar resultados
    results_dir = PROJECT_ROOT / "results"
    results_dir.mkdir(exist_ok=True)
    
    plot_results(
        times=times,
        true_pos=true_positions,
        true_vel=true_velocities,
        ins_pos=ins_positions,
        ins_vel=ins_velocities,
        kf_pos=kf_positions,
        kf_vel=kf_velocities,
        kf_cov=kf_covariances,
        gps_data=gps_data,
        save_path=results_dir / "exp_01_results.png"
    )
    
    print("\nExperimento concluído!")


if __name__ == "__main__":
    main()
