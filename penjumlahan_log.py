import numpy as np

def hitung_total_desibel(daftar_db):
    """
    Ngitung nilai db secara penjumlahan logaritmik
    Standar IEC untuk penjumlahan suara
    """
    db_array = np.array(daftar_db)
    
    # Ubah setiap nilai dB kembali ke skala energi linear (P/P0)^2
    energi_linear = 10 ** (db_array / 10)
    
    # Langkah 2: Jumlahkan semua energi linear tersebut
    total_energi = np.sum(energi_linear)
    #total_energi = (np.sum(energi_linear))/60 kalau mau hitung tiap menit

    # Mencegah error matematika (logaritma dari nol) jika array kosong atau bernilai minus tak terhingga
    if total_energi <= 1e-10:
        return 0.0
        
    # Ubah kembali total energi linear ke skala logaritmik (dB)
    total_db = (10 * np.log10(total_energi))
    
    return total_db

# BAGIAN SIMULASI 

if __name__ == "__main__":
    print(" Simulasi Penjumlahan Logaritmik Desibel ")
    
    # Skenario 1: Dua sumber suara yang persis sama kerasnya
    suara_sama = [60.0, 60.0]
    hasil_1 = hitung_total_desibel(suara_sama)
    print(f"\nSkenario 1: {suara_sama}")
    print(f"Hasil: {hasil_1:.2f} dB")
    # Jika energi suara dikali dua, nilainya akan selalu bertambah ~3 dB.
    
    # Skenario 2: Sumber suara keras digabung dengan suara pelan
    suara_jomplang = [80.0, 82.0, 89.0]
    hasil_2 = hitung_total_desibel(suara_jomplang)
    print(f"\nSkenario 2: {suara_jomplang}")
    print(f"Hasil: {hasil_2:.2f} dB")
    # Suara yang jauh lebih kecil "tenggelam" dan hampir tidak menambah total kebisingan.
    
    # Skenario 3: Simulasi 31 Band Frekuensi
    simulasi_31_band = [
        30, 32, 35, 40, 45, 50, 60, 65, 70, 75, 
        80, 82, 85, 82, 75, 70, 65, 60, 55, 50, 
        45, 40, 35, 30, 25, 20, 20, 20, 20, 20, 20
    ]
    hasil_3 = hitung_total_desibel(simulasi_31_band)
    print(f"\nSkenario 3: Penggabungan 31 Band Frekuensi (Max di {max(simulasi_31_band)} dB)")
    print(f"Total Keseluruhan (Leq Total): {hasil_3:.2f} dB")