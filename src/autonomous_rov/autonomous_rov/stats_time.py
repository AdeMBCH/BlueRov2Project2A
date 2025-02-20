#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# Chemins des fichiers CSV
csv_file_hsv = '/home/projet_sysmer/ros2_ws/detection_timesHSV.csv'
csv_file_yolo = '/home/projet_sysmer/ros2_ws/detection_timesYOLO.csv'

# Chargement des données
data_hsv = pd.read_csv(csv_file_hsv)
data_yolo = pd.read_csv(csv_file_yolo)

# Filtrage des données pour ne garder que celles entre 0 et 200 ms
filtered_hsv = data_hsv[(data_hsv['Iteration'] >= 0) & (data_hsv['Iteration'] <= 200)]
filtered_yolo = data_yolo[(data_yolo['Iteration'] >= 0) & (data_yolo['Iteration'] <= 200)]

# Calcul des statistiques pour HSV
mean_hsv = filtered_hsv['ExecutionTime (ms)'].mean()
std_hsv = filtered_hsv['ExecutionTime (ms)'].std()
min_hsv = filtered_hsv['ExecutionTime (ms)'].min()
max_hsv = filtered_hsv['ExecutionTime (ms)'].max()

# Calcul des statistiques pour YOLO
mean_yolo = filtered_yolo['ExecutionTime (ms)'].mean()
std_yolo = filtered_yolo['ExecutionTime (ms)'].std()
min_yolo = filtered_yolo['ExecutionTime (ms)'].min()
max_yolo = filtered_yolo['ExecutionTime (ms)'].max()

print("Statistiques HSV:")
print(f" - Temps moyen : {mean_hsv:.2f} ms")
print(f" - Écart-type   : {std_hsv:.2f} ms")
print(f" - Temps min    : {min_hsv:.2f} ms")
print(f" - Temps max    : {max_hsv:.2f} ms")
print("\nStatistiques YOLO:")
print(f" - Temps moyen : {mean_yolo:.2f} ms")
print(f" - Écart-type   : {std_yolo:.2f} ms")
print(f" - Temps min    : {min_yolo:.2f} ms")
print(f" - Temps max    : {max_yolo:.2f} ms")

# Création de la figure et tracé des courbes
plt.figure(figsize=(12, 6))
plt.plot(filtered_hsv['Iteration'], filtered_hsv['ExecutionTime (ms)'], marker='o', linestyle='-', color='red', label='HSV')
plt.plot(filtered_yolo['Iteration'], filtered_yolo['ExecutionTime (ms)'], marker='o', linestyle='-', color='blue', label='YOLO')

plt.title("Comparaison des temps d'exécution par itération")
plt.xlabel("Itération")
plt.ylabel("Temps d'exécution (ms)")
plt.ylim(0, 200)
plt.grid(True)
plt.legend()

# Création des zones de texte avec les statistiques pour chaque méthode
stats_text_hsv = (
    f"HSV:\n"
    f"Temps moyen: {mean_hsv:.2f} ms\n"
    f"Écart-type: {std_hsv:.2f} ms\n"
    f"Temps min: {min_hsv:.2f} ms\n"
    f"Temps max: {max_hsv:.2f} ms"
)

stats_text_yolo = (
    f"YOLO:\n"
    f"Temps moyen: {mean_yolo:.2f} ms\n"
    f"Écart-type: {std_yolo:.2f} ms\n"
    f"Temps min: {min_yolo:.2f} ms\n"
    f"Temps max: {max_yolo:.2f} ms"
)

ax = plt.gca()
ax.text(1.02, 0.7, stats_text_hsv, transform=ax.transAxes, fontsize=12,
        verticalalignment='center', bbox=dict(facecolor='white', alpha=0.5))
ax.text(1.02, 0.3, stats_text_yolo, transform=ax.transAxes, fontsize=12,
        verticalalignment='center', bbox=dict(facecolor='white', alpha=0.5))

plt.tight_layout()
plt.show()
