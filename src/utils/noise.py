"""
Módulo de Ruído - Funções utilitárias para geração de ruído.

Este módulo centraliza a geração de ruído para evitar chamadas
espalhadas de np.random.normal pelo código.

Referência Matemática:
    - Ruído branco gaussiano: w ~ N(0, σ²)
    - Viés constante: b (determinístico)
"""

import numpy as np
from typing import Optional


def set_seed(seed: int) -> None:
    """
    Define a semente do gerador de números aleatórios.
    
    Permite reprodutibilidade dos experimentos.
    
    Args:
        seed: Valor inteiro para a semente.
    """
    np.random.seed(seed)


def white_noise(std: float, size: Optional[int] = None) -> np.ndarray:
    """
    Gera ruído branco gaussiano de média zero.
    
    Modelo Matemático:
        w ~ N(0, σ²)
        
    onde σ (std) é o desvio padrão do ruído.
    
    Args:
        std: Desvio padrão do ruído (σ).
        size: Quantidade de amostras. Se None, retorna um escalar.
        
    Returns:
        Amostra(s) de ruído branco gaussiano.
    """
    if size is None:
        return np.random.normal(0, std)
    return np.random.normal(0, std, size)


def add_bias(value: float, bias: float) -> float:
    """
    Adiciona viés constante a um valor.
    
    Modelo Matemático:
        y = x + b
        
    onde b é o viés (bias).
    
    Args:
        value: Valor original.
        bias: Viés a ser adicionado.
        
    Returns:
        Valor com viés adicionado.
    """
    return value + bias


def corrupt_measurement(
    true_value: float,
    bias: float = 0.0,
    noise_std: float = 0.0
) -> float:
    """
    Corrompe uma medição ideal com viés e ruído.
    
    Modelo Matemático:
        z = x + b + w
        
    onde:
        x = valor verdadeiro
        b = viés (sistemático)
        w ~ N(0, σ²) = ruído branco gaussiano
        
    Args:
        true_value: Valor verdadeiro (ideal).
        bias: Viés sistemático.
        noise_std: Desvio padrão do ruído.
        
    Returns:
        Medição corrompida.
    """
    noisy = true_value + bias + white_noise(noise_std)
    return noisy
