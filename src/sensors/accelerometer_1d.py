"""
Simulador de Acelerômetro 1D.

Este módulo simula um acelerômetro com características
realistas de erro: viés constante e ruído branco.

Modelo de Medição:
    a_meas = a_true + b + w
    
onde:
    a_true = aceleração verdadeira [m/s²]
    b = viés do sensor (bias) [m/s²]
    w ~ N(0, σ²) = ruído branco gaussiano

Efeito do Viés no INS:
    O viés causa deriva quadrática na posição:
    Δx(t) ≈ 0.5 * b * t²
    
    Para b = 0.01 m/s² e t = 100s:
    Δx ≈ 50 metros de erro!

Referência: Titterton & Weston, "Strapdown Inertial Navigation Technology"
"""

import numpy as np
from dataclasses import dataclass

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.noise import white_noise


@dataclass
class AccelerometerParams:
    """
    Parâmetros do acelerômetro.
    
    Attributes:
        bias: Viés constante do sensor [m/s²].
        noise_std: Desvio padrão do ruído branco [m/s²].
    """
    bias: float = 0.01        # Viés típico: 10 mg ≈ 0.1 m/s²
    noise_std: float = 0.05   # Ruído típico: 50 μg/√Hz


class Accelerometer1D:
    """
    Simulador de acelerômetro 1D.
    
    Simula as imperfeições típicas de um acelerômetro MEMS:
    - Viés (bias): erro sistemático constante
    - Ruído branco: erro aleatório gaussiano
    """
    
    def __init__(self, params: AccelerometerParams = None):
        """
        Inicializa o acelerômetro.
        
        Args:
            params: Parâmetros do sensor. Se None, usa valores padrão.
        """
        self.params = params or AccelerometerParams()
        
    @property
    def bias(self) -> float:
        """Retorna o viés do sensor."""
        return self.params.bias
    
    @property
    def noise_std(self) -> float:
        """Retorna o desvio padrão do ruído."""
        return self.params.noise_std
    
    def measure(self, true_acceleration: float) -> float:
        """
        Simula uma medição de aceleração.
        
        Modelo:
            a_meas = a_true + bias + w
            w ~ N(0, noise_std²)
            
        Args:
            true_acceleration: Aceleração verdadeira [m/s²].
            
        Returns:
            Medição de aceleração corrompida [m/s²].
        """
        noise = white_noise(self.params.noise_std)
        measurement = true_acceleration + self.params.bias + noise
        return measurement
    
    def measure_batch(self, true_accelerations: np.ndarray) -> np.ndarray:
        """
        Simula múltiplas medições de aceleração.
        
        Args:
            true_accelerations: Array de acelerações verdadeiras.
            
        Returns:
            Array de medições corrompidas.
        """
        n = len(true_accelerations)
        noise = white_noise(self.params.noise_std, n)
        measurements = true_accelerations + self.params.bias + noise
        return measurements
    
    def get_variance(self) -> float:
        """
        Retorna a variância do ruído de medição.
        
        Útil para configurar a matriz Q do Filtro de Kalman.
        
        Returns:
            Variância σ² do ruído.
        """
        return self.params.noise_std ** 2
    
    def describe(self) -> str:
        """Retorna descrição textual dos parâmetros."""
        return (
            f"Acelerômetro 1D:\n"
            f"  Viés: {self.params.bias:.4f} m/s²\n"
            f"  Ruído (σ): {self.params.noise_std:.4f} m/s²"
        )
