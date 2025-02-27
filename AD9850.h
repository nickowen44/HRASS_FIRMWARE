/*
void AD9850_set_frequency(uint32_t freq){
    freq=freq * 4294967295/AD9850_CLOCK;
    digitalWrite(AD9850_CS, LOW);
    SPI.beginTransaction(SPI_AD9850);
    SPI.write32(freq);
    SPI.write(0);
    SPI.endTransaction();
    digitalWrite(AD9850_CS, HIGH);
}

void AD9850_reset(){
    // RESET AD9850
    digitalWrite(AD9850_RST, HIGH);
    delay(1);
    digitalWrite(AD9850_RST, LOW);
    delay(1);
    AD9850_set_frequency(0);
    delay(1);
}

/*

/*
//The setup function is called once at startup of the sketch
void setup(){
    pinMode(AD9850_CS,  OUTPUT); digitalWrite(AD9850_CS, HIGH);
    pinMode(AD9850_RST, OUTPUT); digitalWrite(AD9850_RST, LOW);

    SPI.begin();
    SPI_AD9850=SPISettings(1000000, LSBFIRST, SPI_MODE0);
    AD9850_reset();

    AD9850_set_frequency(100);
    delay(1000);


}
*/
