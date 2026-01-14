"""
Filtro de Kalman 1D - Fusão INS/GPS.

Este módulo implementa o Filtro de Kalman discreto para
estimação de posição e velocidade em 1D.

Estado do Sistema:
    x = [posição, velocidade]ᵀ

╔══════════════════════════════════════════════════════════════╗
║                    ALGORITMO DO FILTRO                       ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  ┌─────────────── PREDIÇÃO (com aceleração) ───────────────┐ ║
║  │                                                          │ ║
║  │  x̂⁻[k] = F * x̂[k-1] + B * a[k]                          │ ║
║  │  P⁻[k] = F * P[k-1] * Fᵀ + Q                            │ ║
║  │                                                          │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                           │                                  ║
║                           ▼                                  ║
║  ┌─────────────── CORREÇÃO (com GPS) ──────────────────────┐ ║
║  │                                                          │ ║
║  │  K[k] = P⁻[k] * Hᵀ * (H * P⁻[k] * Hᵀ + R)⁻¹            │ ║
║  │  x̂[k] = x̂⁻[k] + K[k] * (z[k] - H * x̂⁻[k])              │ ║
║  │  P[k] = (I - K[k] * H) * P⁻[k]                          │ ║
║  │                                                          │ ║
║  └──────────────────────────────────────────────────────────┘ ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝

Matrizes:
    F = [[1, dt], [0, 1]]     - Transição de estado
    B = [[0.5*dt²], [dt]]     - Matriz de controle
    H = [[1, 0]]              - Observação (posição)
    Q                         - Covariância do ruído de processo
    R                         - Covariância do ruído de medição

Referência: Welch & Bishop, "An Introduction to the Kalman Filter"
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class KalmanState:
    """
    Estado interno do Filtro de Kalman.
    
    Attributes:
        x: Vetor de estado estimado [posição, velocidade]ᵀ.
        P: Matriz de covariância do erro de estimação (2x2).
    """
    x: np.ndarray  # (2, 1)
    P: np.ndarray  # (2, 2)
    
    @property
    def position(self) -> float:
        """Retorna a posição estimada."""
        return self.x[0, 0]
    
    @property
    def velocity(self) -> float:
        """Retorna a velocidade estimada."""
        return self.x[1, 0]
    
    @property
    def position_variance(self) -> float:
        """Retorna a variância da posição."""
        return self.P[0, 0]
    
    @property
    def velocity_variance(self) -> float:
        """Retorna a variância da velocidade."""
        return self.P[1, 1]


class KalmanFilter1D:
    """
    Filtro de Kalman 1D para fusão INS/GPS.
    
    Modelo:
        Estado: x = [posição, velocidade]ᵀ
        Entrada de controle: u = aceleração medida
        Medição: z = posição GPS
        
    O filtro:
        1. PREDIZ o estado usando a aceleração (INS)
        2. CORRIGE usando a posição GPS
    """
    
    def __init__(
        self,
        dt: float,
        process_noise_std: float,
        measurement_noise_std: float,
        initial_position: float = 0.0,
        initial_velocity: float = 0.0,
        initial_covariance: float = 1.0
    ):
        """
        Inicializa o Filtro de Kalman.
        
        Args:
            dt: Intervalo de tempo entre amostras [s].
            process_noise_std: Desvio padrão do ruído de processo.
            measurement_noise_std: Desvio padrão do ruído de medição (GPS).
            initial_position: Estimativa inicial de posição [m].
            initial_velocity: Estimativa inicial de velocidade [m/s].
            initial_covariance: Valor inicial para diagonal de P.
        """
        self.dt = dt
        
        # Matrizes do sistema
        self._F = self._build_F(dt)
        self._B = self._build_B(dt)
        self._H = np.array([[1.0, 0.0]])  # Mede apenas posição
        
        # Covariâncias de ruído
        self._Q = self._build_Q(dt, process_noise_std)
        self._R = np.array([[measurement_noise_std ** 2]])
        
        # Estado inicial
        self._state = KalmanState(
            x=np.array([[initial_position], [initial_velocity]]),
            P=np.eye(2) * initial_covariance
        )
        
        # Histórico para análise
        self._history = {
            'x': [],
            'P': [],
            'K': []
        }
    
    @staticmethod
    def _build_F(dt: float) -> np.ndarray:
        """
        Constrói a matriz de transição de estado.
        
            F = [[1, dt],
                 [0,  1]]
        """
        return np.array([
            [1.0, dt],
            [0.0, 1.0]
        ])
    
    @staticmethod
    def _build_B(dt: float) -> np.ndarray:
        """
        Constrói a matriz de controle.
        
            B = [[0.5*dt²],
                 [dt     ]]
        """
        return np.array([
            [0.5 * dt ** 2],
            [dt]
        ])
    
    @staticmethod
    def _build_Q(dt: float, sigma_a: float) -> np.ndarray:
        """
        Constrói a matriz de covariância do ruído de processo.
        
        Modelo: ruído de aceleração como ruído de processo.
        
            Q = G * σ_a² * Gᵀ
            
        onde G = B (matriz de controle).
        
        Resultado:
            Q = [[dt⁴/4, dt³/2],
                 [dt³/2, dt²  ]] * σ_a²
        """
        dt2 = dt ** 2
        dt3 = dt ** 3
        dt4 = dt ** 4
        
        Q = np.array([
            [dt4 / 4, dt3 / 2],
            [dt3 / 2, dt2]
        ]) * (sigma_a ** 2)
        
        return Q
    
    @property
    def state(self) -> KalmanState:
        """Retorna o estado atual do filtro."""
        return self._state
    
    @property
    def F(self) -> np.ndarray:
        """Matriz de transição de estado."""
        return self._F
    
    @property
    def B(self) -> np.ndarray:
        """Matriz de controle."""
        return self._B
    
    @property
    def H(self) -> np.ndarray:
        """Matriz de observação."""
        return self._H
    
    @property
    def Q(self) -> np.ndarray:
        """Matriz de covariância do ruído de processo."""
        return self._Q
    
    @property
    def R(self) -> np.ndarray:
        """Matriz de covariância do ruído de medição."""
        return self._R
    
    def predict(self, acceleration: float) -> KalmanState:
        """
        Etapa de PREDIÇÃO do Filtro de Kalman.
        
        Propaga o estado e a covariância usando o modelo dinâmico.
        
        Equações:
            x̂⁻ = F * x̂ + B * a
            P⁻ = F * P * Fᵀ + Q
            
        Args:
            acceleration: Aceleração medida pelo acelerômetro [m/s²].
            
        Returns:
            Estado predito (a priori).
        """
        # Estado predito
        u = np.array([[acceleration]])
        x_pred = self._F @ self._state.x + self._B @ u
        
        # Covariância predita
        P_pred = self._F @ self._state.P @ self._F.T + self._Q
        
        # Atualiza estado interno
        self._state = KalmanState(x=x_pred, P=P_pred)
        
        return self._state
    
    def update(self, gps_position: float) -> Tuple[KalmanState, np.ndarray]:
        """
        Etapa de CORREÇÃO do Filtro de Kalman.
        
        Incorpora a medição GPS para corrigir o estado predito.
        
        Equações:
            S = H * P⁻ * Hᵀ + R           (inovação)
            K = P⁻ * Hᵀ * S⁻¹             (ganho de Kalman)
            y = z - H * x̂⁻                (resíduo)
            x̂ = x̂⁻ + K * y               (estado corrigido)
            P = (I - K * H) * P⁻          (covariância corrigida)
            
        Args:
            gps_position: Posição medida pelo GPS [m].
            
        Returns:
            Tupla (estado corrigido, ganho de Kalman K).
        """
        z = np.array([[gps_position]])
        
        # Covariância da inovação
        S = self._H @ self._state.P @ self._H.T + self._R
        
        # Ganho de Kalman
        K = self._state.P @ self._H.T @ np.linalg.inv(S)
        
        # Resíduo (inovação)
        y = z - self._H @ self._state.x
        
        # Estado corrigido
        x_upd = self._state.x + K @ y
        
        # Covariância corrigida (forma Joseph para estabilidade)
        I = np.eye(2)
        P_upd = (I - K @ self._H) @ self._state.P
        
        # Atualiza estado interno
        self._state = KalmanState(x=x_upd, P=P_upd)
        
        # Salva histórico
        self._history['x'].append(x_upd.copy())
        self._history['P'].append(P_upd.copy())
        self._history['K'].append(K.copy())
        
        return self._state, K
    
    def step(
        self,
        acceleration: float,
        gps_position: float = None
    ) -> KalmanState:
        """
        Executa um passo completo do filtro.
        
        1. Predição usando aceleração
        2. Correção usando GPS (se disponível)
        
        Args:
            acceleration: Aceleração medida [m/s²].
            gps_position: Posição GPS [m]. Se None, apenas predição.
            
        Returns:
            Estado atual após o passo.
        """
        self.predict(acceleration)
        
        if gps_position is not None:
            self.update(gps_position)
            
        return self._state
    
    def get_estimates(self) -> Tuple[float, float]:
        """
        Retorna as estimativas atuais.
        
        Returns:
            Tupla (posição_estimada, velocidade_estimada).
        """
        return self._state.position, self._state.velocity
    
    def get_uncertainties(self) -> Tuple[float, float]:
        """
        Retorna as incertezas atuais (1σ).
        
        Returns:
            Tupla (σ_posição, σ_velocidade).
        """
        return (
            np.sqrt(self._state.position_variance),
            np.sqrt(self._state.velocity_variance)
        )
    
    def describe(self) -> str:
        """Retorna descrição textual do estado atual."""
        pos, vel = self.get_estimates()
        sig_pos, sig_vel = self.get_uncertainties()
        
        return (
            f"Filtro de Kalman 1D:\n"
            f"  Posição: {pos:.2f} ± {sig_pos:.2f} m\n"
            f"  Velocidade: {vel:.2f} ± {sig_vel:.2f} m/s\n"
            f"  dt: {self.dt:.3f} s"
        )
