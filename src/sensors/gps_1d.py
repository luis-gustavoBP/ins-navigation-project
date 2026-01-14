"""
Simulador de GPS 1D.

Este módulo simula um receptor GPS simplificado que
fornece apenas medições de posição em 1D.

Modelo de Medição:
    z_gps = x_true + w
    
onde:
    x_true = posição verdadeira [m]
    w ~ N(0, σ²) = ruído branco gaussiano

Características do GPS:
    - Alta precisão absoluta (sem deriva)
    - Ruído relativamente alto comparado ao acelerômetro
    - Taxa de amostragem tipicamente baixa (1-10 Hz)
    - Sem viés significativo (após correções atmosféricas)

Precisão Típica:
    - GPS civil: σ ≈ 3-5 metros
    - GPS diferencial (DGPS): σ ≈ 0.5-2 metros
    - RTK GPS: σ ≈ 0.01-0.02 metros

Referência: Kaplan & Hegarty, "Understanding GPS: Principles and Applications"
"""

import numpy as np
from dataclasses import dataclass

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.noise import white_noise


@dataclass
class GPSParams:
    """
    Parâmetros do GPS.
    
    Attributes:
        noise_std: Desvio padrão do ruído de posição [m].
        sample_rate: Taxa de amostragem [Hz] (para referência).
    """
    noise_std: float = 3.0    # GPS civil típico: 3 metros
    sample_rate: float = 1.0  # 1 Hz é comum para GPS civil


class GPS1D:
    """
    Simulador de GPS 1D.
    
    Simula um receptor GPS que mede apenas posição.
    
    Características modeladas:
    - Ruído branco gaussiano (após correções)
    - Sem viés (hipótese simplificadora)
    - Medição direta de posição (não velocidade)
    """
    
    def __init__(self, params: GPSParams = None):
        """
        Inicializa o GPS.
        
        Args:
            params: Parâmetros do sensor. Se None, usa valores padrão.
        """
        self.params = params or GPSParams()
        
    @property
    def noise_std(self) -> float:
        """Retorna o desvio padrão do ruído."""
        return self.params.noise_std
    
    def measure(self, true_position: float) -> float:
        """
        Simula uma medição de posição GPS.
        
        Modelo:
            z = x_true + w
            w ~ N(0, noise_std²)
            
        Args:
            true_position: Posição verdadeira [m].
            
        Returns:
            Medição de posição corrompida [m].
        """
        noise = white_noise(self.params.noise_std)
        measurement = true_position + noise
        return measurement
    
    def measure_batch(self, true_positions: np.ndarray) -> np.ndarray:
        """
        Simula múltiplas medições de posição.
        
        Args:
            true_positions: Array de posições verdadeiras.
            
        Returns:
            Array de medições corrompidas.
        """
        n = len(true_positions)
        noise = white_noise(self.params.noise_std, n)
        measurements = true_positions + noise
        return measurements
    
    def get_variance(self) -> float:
        """
        Retorna a variância do ruído de medição.
        
        Útil para configurar a matriz R do Filtro de Kalman.
        
        Returns:
            Variância σ² do ruído.
        """
        return self.params.noise_std ** 2
    
    def observation_matrix(self) -> np.ndarray:
        """
        Retorna a matriz de observação H.
        
        Para GPS medindo apenas posição:
            z = H * x = [1, 0] * [x, v]ᵀ = x
            
        Returns:
            Matriz H (1x2).
        """
        return np.array([[1.0, 0.0]])
    
    def describe(self) -> str:
        """Retorna descrição textual dos parâmetros."""
        return (
            f"GPS 1D:\n"
            f"  Ruído (σ): {self.params.noise_std:.2f} m\n"
            f"  Taxa: {self.params.sample_rate:.1f} Hz"
        )
