# test: emulating the ws2812b protocol via SPI

## what

- a bluepill (currently)
- a strip of ws2812b leds

## why

bit-banging the protocol in software eats up processor resources, and maybe you'd prefer spending those in your rendering routine. on the other hand, the spi peripheral can be fed directly from main memory using dma, leaving the processor free to prepare the next frame.

# how

according to several versions of the [datasheet][https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf] floating around, the ws2812b protocol encodes a 0 bit as 0.4µs high followed by 0.85µs low, and a 1 bit as 0.8µs high followed by 0.45µs low.

now, these values *almost* fall neatly into categories of *one unit long* and *two units long*; in fact, the specified tolerances are loose enough to allow this *almost* to take on values of *exactly*, given appropriate definitions of *unit*.

in reality, the tolerances are looser than those in the datasheet and thus appear to permit *unit*=0.5µs, despite the fact that this is outside specified tolerances for T1H by 50ns. this is convenient, because configuring the spi clock to 0.5µs/bit, or 2Mbit/s, is pretty easy. (we're working on a better solution though to get 2.5Mbit/s, or 0.4µs/bit, which is basically perfect)

(also apparently there is a [different datasheet][http://www.world-semi.com/DownLoadFile/108] directly from the worldsemi website, which has kind of different values including a longer low period for the 1 bit, a shorter high period for the 0 bit, and a much longer minimum low time for resetting. what?)

anyway, let's go back to the neat model we had above. since spi pushes out exactly one bit per clock cycle, we can emulate those characteristics simply by encoding a 0 as `100` and a 1 as `110`. (take a look at `write_byte` in `Core/Src/main.c`, that's exactly what it does.) using this encoding, we fill a memory buffer with the appropriate amount of bits to reach all leds, plus some zeros at the end to enforce the minimum reset time. then we start the dma and let it do its job while computing the next frame.

there are some specifics where if CPHA is 1 (the default), meaning that each bit is sent on the first edge of the clock (so, the rising edge if the clock is normally zero), the first bit will be held for two cycles instead of just one, which messes things up. (i guess this makes sense for spi though, since it wants the bit to be already set when the clock goes high for the first time?) anyway, it's possible to circumvent this problem by making sure the first bit is always zero (for us, the spi should always be zero before the next dma, since the previous one ended with a run of zeros), which means that no-one will notice there's an extra one. realistically, it's easier to send a whole 0 byte, but that doesn't matter either. alternatively (and this is what we do currently), you can just set CPHA to 0, which means spi writes each bit on the second edge of the clock, and the problem disappears.
