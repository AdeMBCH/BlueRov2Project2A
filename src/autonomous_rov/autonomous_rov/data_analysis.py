#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

# --- 1. Lecture du fichier CSV ---
df = pd.read_csv('/home/projet_sysmer/ros2_ws/tracking_data.csv')

# --- 2. Création de la colonne "time" réinitialisée pour chaque méthode ---
df['time'] = df.groupby('methode').cumcount()

# --- 3. Filtrage des données pour n'utiliser que celles de time entre 25 et 90 ---
df_filtered = df[(df['time'] >= 25) & (df['time'] <= 70)].copy()

# Conversion des colonnes numériques si nécessaire
df_filtered['segment_width'] = pd.to_numeric(df_filtered['segment_width'], errors='coerce')
df_filtered['pos_x'] = pd.to_numeric(df_filtered['pos_x'], errors='coerce')
df_filtered['pos_y'] = pd.to_numeric(df_filtered['pos_y'], errors='coerce')

# -----------------------------------------------------
# Fenêtre 1 : pos_x vs time
# -----------------------------------------------------
plt.figure(figsize=(10, 5))
for methode in df_filtered['methode'].unique():
    df_m = df_filtered[df_filtered['methode'] == methode]
    plt.plot(df_m['time'], df_m['pos_x'], label=f'{methode} pos_x', marker='o')
plt.title('Évolution de pos_x')
plt.xlabel('Time (index réinitialisé par méthode)')
plt.ylabel('Position X')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# -----------------------------------------------------
# Fenêtre 2 : pos_y vs time
# -----------------------------------------------------
plt.figure(figsize=(10, 5))
for methode in df_filtered['methode'].unique():
    df_m = df_filtered[df_filtered['methode'] == methode]
    plt.plot(df_m['time'], df_m['pos_y'], label=f'{methode} pos_y', marker='o')
plt.title('Évolution de pos_y')
plt.xlabel('Time (index réinitialisé par méthode)')
plt.ylabel('Position Y')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# -----------------------------------------------------
# Fenêtre 3 : segment_width vs time
# -----------------------------------------------------
plt.figure(figsize=(10, 5))
for methode in df_filtered['methode'].unique():
    df_m = df_filtered[df_filtered['methode'] == methode]
    plt.plot(df_m['time'], df_m['segment_width'], label=f'{methode} segment_width', marker='o')
plt.title('Évolution de la largeur de segment')
plt.xlabel('Time (index réinitialisé par méthode)')
plt.ylabel('Segment Width')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# -----------------------------------------------------
# Calcul des statistiques
# -----------------------------------------------------
stats = df_filtered.groupby('methode').agg({
    'segment_width': ['mean', 'std', 'min', 'max'],
    'pos_x': ['mean', 'std', 'min', 'max'],
    'pos_y': ['mean', 'std', 'min', 'max']
})
print("Statistiques par méthode :")
print(stats)

# -----------------------------------------------------
# Fenêtre 4 : barres pour les statistiques (moyenne ± std)
# -----------------------------------------------------
measures = ['segment_width', 'pos_x', 'pos_y']
fig_stats, axes_stats = plt.subplots(1, len(measures), figsize=(18, 6))

for i, measure in enumerate(measures):
    means = stats[measure]['mean']
    stds = stats[measure]['std']
    axes_stats[i].bar(means.index, means, yerr=stds, capsize=5, color=['skyblue', 'salmon'])
    axes_stats[i].set_title(f"Moyenne de {measure} ± std")
    axes_stats[i].set_ylabel(measure)
    axes_stats[i].grid(True)

plt.suptitle("Statistiques sur les données filtrées (time entre 25 et 90)")
plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()
