"""
Modelo Físico de Movimento 1D - Modelo ideal (sem ruído).

Este módulo implementa as equações cinemáticas do movimento
retilíneo uniformemente variado (MRUV) em forma discretizada.

Equações Contínuas:
    dx/dt = v(t)
    dv/dt = a(t)

Equações Discretizadas (Euler):
    x[k+1] = x[k] + v[k]*dt + 0.5*a[k]*dt²
    v[k+1] = v[k] + a[k]*dt

Referência: Mecânica Clássica - Movimento em 1D
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class State1D:
    """
    Estado do sistema em 1D.
    
    Attributes:
        position: Posição em metros [m].
        velocity: Velocidade em metros por segundo [m/s].
    """
    position: float
    velocity: float
    
    def to_vector(self) -> np.ndarray:
        """Retorna o estado como vetor coluna [x, v]ᵀ."""
        return np.array([[self.position], [self.velocity]])
    
    @classmethod
    def from_vector(cls, vec: np.ndarray) -> 'State1D':
        """Cria State1D a partir de um vetor [x, v]ᵀ."""
        return cls(position=vec[0, 0], velocity=vec[1, 0])


class MotionModel1D:
    """
    Modelo de movimento 1D ideal (determinístico).
    
    Implementa as equações cinemáticas discretizadas para
    movimento com aceleração arbitrária.
    
    Modelo de Estado:
        x = [posição, velocidade]ᵀ
        
    Equação de Transição:
        x[k+1] = F * x[k] + B * u[k]
        
    onde:
        F = [[1, dt], [0, 1]]  (matriz de transição)
        B = [[0.5*dt²], [dt]]  (matriz de controle)
        u = a (aceleração como entrada de controle)
    """
    
    def __init__(self, dt: float):
        """
        Inicializa o modelo de movimento.
        
        Args:
            dt: Intervalo de tempo entre amostras [s].
        """
        self.dt = dt
        
    def state_transition_matrix(self) -> np.ndarray:
        """
        Retorna a matriz de transição de estado F.
        
        F = [[1, dt],
             [0,  1]]
             
        Returns:
            Matriz F (2x2).
        """
        return np.array([
            [1.0, self.dt],
            [0.0, 1.0]
        ])
    
    def control_matrix(self) -> np.ndarray:
        """
        Retorna a matriz de controle B.
        
        B = [[0.5*dt²],
             [dt     ]]
             
        Returns:
            Matriz B (2x1).
        """
        return np.array([
            [0.5 * self.dt**2],
            [self.dt]
        ])
    
    def predict(self, state: State1D, acceleration: float) -> State1D:
        """
        Propaga o estado para o próximo instante de tempo.
        
        Equações:
            x[k+1] = x[k] + v[k]*dt + 0.5*a*dt²
            v[k+1] = v[k] + a*dt
            
        Args:
            state: Estado atual.
            acceleration: Aceleração aplicada [m/s²].
            
        Returns:
            Novo estado após dt segundos.
        """
        F = self.state_transition_matrix()
        B = self.control_matrix()
        
        x = state.to_vector()
        u = np.array([[acceleration]])
        
        x_new = F @ x + B @ u
        
        return State1D.from_vector(x_new)
    
    def simulate_trajectory(
        self,
        initial_state: State1D,
        accelerations: List[float]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simula uma trajetória completa dado um perfil de aceleração.
        
        Args:
            initial_state: Estado inicial do sistema.
            accelerations: Lista de acelerações para cada instante.
            
        Returns:
            Tupla (tempo, posições, velocidades).
        """
        n_steps = len(accelerations)
        
        times = np.arange(n_steps + 1) * self.dt
        positions = np.zeros(n_steps + 1)
        velocities = np.zeros(n_steps + 1)
        
        positions[0] = initial_state.position
        velocities[0] = initial_state.velocity
        
        state = initial_state
        for k, acc in enumerate(accelerations):
            state = self.predict(state, acc)
            positions[k + 1] = state.position
            velocities[k + 1] = state.velocity
            
        return times, positions, velocities
