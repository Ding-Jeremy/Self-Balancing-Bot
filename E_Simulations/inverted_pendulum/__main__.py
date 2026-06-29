#   name:           inverted_pendulum
#   author:         Ding Jérémy
#   description:    This module serves as a simulation in order to observe the physics behind
#                   an Inverted pendulum
import pygame
import math
import sys


# ========================
# Physical Parameters
# ========================
M = 1.5  # cart mass
m = 0.5  # pendulum mass
l = 0.5  # pendulum length (meters)
g = 9.81
v_target = 0  # desired speed (m/s)
theta_max = 0.2  # limit lean angle (radians ~ 11°)
F_max = 10
# ========================
# PID Gains (start small)
# ========================
Kp = 100
Kd = 50
Ki = 0
Kv_speed = 0.5  # velocity loop gain

# ========================
# Initial State
# ========================
x = 0.0
x_dot = 0.0
theta = 0.2
theta_dot = 0.0
integral_error = 0

# ========================
# Simulation Settings
# ========================
dt = 0.01
scale = 200  # meters → pixels

# ========================
# Pygame Setup
# ========================
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

# ========================
# Main Loop
# ========================

running = True
while running:
    clock.tick(240)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ========================
    # Set velocity target based on key press
    # ========================
    keys = pygame.key.get_pressed()  # get current key states

    if keys[pygame.K_RIGHT]:
        v_target = 1.5  # forward speed
    elif keys[pygame.K_LEFT]:
        v_target = -1.5  # backward speed
    else:
        v_target = 0.0  # stop if no key pressed

    # ========================
    # Speed Control (Outer Loop)
    # ========================
    velocity_error = v_target - x_dot
    theta_ref = Kv_speed * velocity_error

    # limit reference angle
    theta_ref = max(min(theta_ref, theta_max), -theta_max)

    # ========================
    # Angle PID (Inner Loop)
    # ========================
    error = theta - theta_ref
    integral_error += error * dt
    derivative = theta_dot

    F = -Kp * error - Kd * derivative - Ki * integral_error
    F *= -1
    # Force dampening (friction due to speed)
    F -= 1 * x_dot
    # Torque limit
    F = max(min(F, F_max), -F_max)

    # ========================
    # Dynamics
    # ========================
    denom = M + m - m * math.cos(theta) ** 2
    num = (
        F
        + m * l * math.sin(theta) * theta_dot**2
        - m * g * math.sin(theta) * math.cos(theta)
    )
    x_ddot = num / denom

    theta_ddot = (g * math.sin(theta) - math.cos(theta) * x_ddot) / l

    # ========================
    # Integrate (Euler method)
    # ========================
    x += x_dot * dt
    x_dot += x_ddot * dt
    theta_dot += theta_ddot * dt
    theta += theta_dot * dt

    # ========================
    # Drawing
    # ========================
    screen.fill((150, 150, 150))

    cart_y = 400
    cart_x = 400 + x * scale

    # Draw cart
    pygame.draw.rect(screen, (50, 50, 50), (cart_x - 40, cart_y - 20, 80, 40), 2)

    # Pendulum end position
    pend_x = cart_x + l * scale * math.sin(theta)
    pend_y = cart_y - l * scale * math.cos(theta)

    # Draws elements

    pygame.draw.line(screen, (50, 50, 50), (cart_x, cart_y), (pend_x, pend_y), 5)

    pygame.draw.circle(screen, (50, 50, 50), (int(pend_x), int(pend_y)), 10)

    pygame.display.flip()

pygame.quit()
sys.exit()
