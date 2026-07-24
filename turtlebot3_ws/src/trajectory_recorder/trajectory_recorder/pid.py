"""
pid.py
------
Implementación simple y reutilizable de un controlador PID, usada por el
player.py para corregir la velocidad lineal y angular del robot en base
al error entre la trayectoria grabada y la posición/orientación real
reportada por la odometría.

Incluye:
* Término proporcional, integral y derivativo.
* Anti-windup (saturación del término integral).
* Límite de salida configurable.
* Método reset() para reiniciar el estado entre reproducciones.
"""

from dataclasses import dataclass


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


class PIDController:
    """
    Controlador PID de un solo eje (se instancia una vez por variable a
    controlar, p. ej. una instancia para el error de distancia y otra
    para el error angular).
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 1.0,
        integral_limit: float = 1.0,
    ):
        self.gains = PIDGains(kp, ki, kd)
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_update = True

    def reset(self) -> None:
        """Reinicia el estado interno del controlador (integral y derivada)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_update = True

    def update(self, error: float, dt: float) -> float:
        """
        Calcula la salida de control para un error dado y un paso de
        tiempo dt (segundos). Debe llamarse periódicamente con dt > 0.
        """
        if dt <= 0.0:
            return 0.0

        # Término proporcional
        p_term = self.gains.kp * error

        # Término integral (con anti-windup por saturación)
        self._integral += error * dt
        self._integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral)
        )
        i_term = self.gains.ki * self._integral

        # Término derivativo (evita salto brusco en la primera llamada)
        if self._first_update:
            derivative = 0.0
            self._first_update = False
        else:
            derivative = (error - self._prev_error) / dt
        d_term = self.gains.kd * derivative
        self._prev_error = error

        output = p_term + i_term + d_term

        # Saturación de la salida
        output = max(-self.output_limit, min(self.output_limit, output))
        return output

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Permite actualizar las ganancias en caliente (p. ej. vía parámetros)."""
        self.gains = PIDGains(kp, ki, kd)
