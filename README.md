# RS485 16x Relay Expander

Ovaj projekat predstavlja adresirani RS485 ekspander za upravljanje do 16 releja putem serijske komunikacije. Namenjen je za automatizaciju, industrijske sisteme, pametne kuće i druge aplikacije gde je potrebno pouzdano i fleksibilno upravljanje velikim brojem releja na daljinu.

## Karakteristike

- **Broj releja:** 16
- **Komunikacija:** RS485 (Modbus ili custom protokol)
- **Napajanje:** 12V DC
- **Maksimalna struja kontakta:**  10A
- **Adresabilnost:** Svaka ploča može imati sopstvenu adresu na magistrali
- **Indikacija:** LED indikatori za stanje svakog releja

## Upotreba

1. **Povezivanje:** Povežite ekspander na RS485 magistralu i napajanje.
2. **Konfiguracija adrese:** (Unesite postupak podešavanja adrese, DIP prekidači, softverski itd.)
3. **Komunikacija:** Koristite Modbus RTU ili prilagođeni serijski protokol za upravljanje relejima.
4. **Komande:** Primeri serijskih komandi za uključivanje/isključivanje releja.

## Primer serijske komande

```
<adresa> <komanda> <relej> <stanje>
```
- adresa: adresa ekspandera na magistrali
- komanda: SET
- relej: broj releja (1-16)
- stanje: ON/OFF

## Hardver

- [Shema povezivanja](shema.pdf)  (https://github.com/eldar6776/RelayExpander/blob/main/hw/DE-260124/DE-260124.pdf)
- [PCB dizajn](pcb_files/)    ([hw](https://github.com/eldar6776/RelayExpander/tree/main/hw))

## Softver

- Primer Modbus biblioteke za Arduino/Python/C
- Firmware za mikrokontroler (ako postoji)

## Instalacija

1. Preuzmite ili klonirajte repozitorijum:
   ```
   git clone https://github.com/eldar6776/RelayExpander.git
   ```
2. Pratite uputstva za montažu i povezivanje u dokumentaciji.

## Kontakt

Za dodatne informacije, predloge ili izveštaje o greškama, kontaktirajte autora putem [GitHub Issues](https://github.com/eldar6776/RelayExpander/issues) ili emaila.

---

**Napomena:** Ovaj projekat je open-source. Slobodno ga koristite, modifikujte i doprinosite razvoju!
