import pandas as pd

def excel_to_vcf(excel_file, sheet_name=0, name_col="Name", phone_col="Phone", output_file="contacts.vcf"):
    # Excel dosyasını oku
    df = pd.read_excel(excel_file, sheet_name=sheet_name, dtype=str)
    
    # Tek bir VCF dosyası oluştur
    vcf_content = ""
    
    for index, row in df.iterrows():
        name = row.get(name_col, "Unknown")
        phone = row.get(phone_col, "")
        
        if pd.isna(name) or pd.isna(phone):
            print("Atlandı")
            continue  # Eksik veri varsa atla
        
        name_parts = name.rsplit(" ", 1)
        if len(name_parts) > 1:
            name = f"PGV {name_parts[0]} {name_parts[1].upper()}"
        else:
            name = f"PGV {name.upper()}"
        
        vcard_entry = f"""
BEGIN:VCARD
VERSION:3.0
FN:{name}
TEL;TYPE=CELL:{phone}
END:VCARD
""".strip()
        
        vcf_content += vcard_entry + "\n"
    
    with open(output_file, "w", encoding="utf-8") as vcf_file:
        vcf_file.write(vcf_content)
    
    print(f"Tüm VCF kayıtları '{output_file}' dosyasına kaydedildi.")

# Kullanım
dosya_adi = "kisiler.xlsx"  # Excel dosyanızın adı
excel_to_vcf(dosya_adi)
