library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pdm_mic_monitor is
    port (
        clk100     : in  std_logic;
        resetn     : in  std_logic;

        mic_data   : in  std_logic;
        mic_clk    : out std_logic;
        mic_lrsel  : out std_logic;

        level      : out std_logic_vector(15 downto 0);

        -- New PCM debug outputs
        -- Signed 16 bit PCM in two's complement format.
        pcm_sample : out std_logic_vector(15 downto 0);
        pcm_valid  : out std_logic
    );
end pdm_mic_monitor;

architecture rtl of pdm_mic_monitor is

    -- 100 MHz / (2 * 20) = 2.5 MHz microphone clock.
    -- This is close enough to the usual PDM microphone clock range.
    constant CLK_DIV_MAX : unsigned(5 downto 0) := to_unsigned(19, 6);

    -- PCM path:
    -- 2.5 MHz / 156 = about 16.03 kHz.
    -- This gives a practical near 16 kHz PCM sample rate.
    constant PCM_DECIM_MAX : unsigned(7 downto 0) := to_unsigned(155, 8);
    constant PCM_CENTER    : integer := 78;
    constant PCM_SHIFT     : natural := 8;

    -- Existing level path:
    -- Keep the old 1024 bit window for backward compatibility.
    constant LEVEL_WINDOW_MAX : unsigned(9 downto 0) := to_unsigned(1023, 10);
    constant LEVEL_CENTER     : integer := 512;

    -- Stretch pcm_valid so MicroBlaze can see it through AXI GPIO during debug.
    -- 5000 cycles at 100 MHz = 50 us.
    -- PCM period is about 62.4 us.
    constant VALID_STRETCH_MAX : unsigned(12 downto 0) := to_unsigned(4999, 13);

    signal div_cnt      : unsigned(5 downto 0) := (others => '0');
    signal mic_clk_reg  : std_logic := '0';

    -- Old level output state
    signal level_cnt       : unsigned(9 downto 0) := (others => '0');
    signal level_ones_cnt  : unsigned(10 downto 0) := (others => '0');
    signal level_reg       : unsigned(15 downto 0) := (others => '0');

    -- New PCM output state
    signal pcm_cnt          : unsigned(7 downto 0) := (others => '0');
    signal pcm_ones_cnt     : unsigned(7 downto 0) := (others => '0');
    signal pcm_sample_reg   : signed(15 downto 0) := (others => '0');
    signal pcm_valid_reg    : std_logic := '0';
    signal valid_stretch_cnt: unsigned(12 downto 0) := (others => '0');

begin

    mic_clk    <= mic_clk_reg;
    mic_lrsel  <= '0';

    level      <= std_logic_vector(level_reg);

    pcm_sample <= std_logic_vector(pcm_sample_reg);
    pcm_valid  <= pcm_valid_reg;

    process(clk100)
        variable pdm_rise_v        : boolean;

        variable level_ones_next   : unsigned(10 downto 0);
        variable level_centered_v  : signed(11 downto 0);
        variable level_magnitude_v : unsigned(11 downto 0);

        variable pcm_ones_next     : unsigned(7 downto 0);
        variable pcm_centered_v    : signed(9 downto 0);
        variable pcm_scaled_v      : signed(15 downto 0);
    begin
        if rising_edge(clk100) then

            pdm_rise_v := false;

            if resetn = '0' then
                div_cnt           <= (others => '0');
                mic_clk_reg       <= '0';

                level_cnt         <= (others => '0');
                level_ones_cnt    <= (others => '0');
                level_reg         <= (others => '0');

                pcm_cnt           <= (others => '0');
                pcm_ones_cnt      <= (others => '0');
                pcm_sample_reg    <= (others => '0');
                pcm_valid_reg     <= '0';
                valid_stretch_cnt <= (others => '0');

            else

                -- Stretch pcm_valid for GPIO debug visibility.
                if valid_stretch_cnt = 0 then
                    pcm_valid_reg <= '0';
                else
                    pcm_valid_reg <= '1';
                    valid_stretch_cnt <= valid_stretch_cnt - 1;
                end if;

                -- Generate microphone clock.
                if div_cnt = CLK_DIV_MAX then
                    div_cnt <= (others => '0');

                    -- Detect the generated rising edge.
                    if mic_clk_reg = '0' then
                        pdm_rise_v := true;
                    end if;

                    mic_clk_reg <= not mic_clk_reg;
                else
                    div_cnt <= div_cnt + 1;
                end if;

                -- Sample PDM data on generated mic clock rising edge.
                if pdm_rise_v then

                    --------------------------------------------------------
                    -- Existing level path, kept compatible with old design
                    --------------------------------------------------------
                    level_ones_next := level_ones_cnt;

                    if mic_data = '1' then
                        level_ones_next := level_ones_cnt + 1;
                    end if;

                    if level_cnt = LEVEL_WINDOW_MAX then

                        -- Center around 512 ones per 1024 bits.
                        level_centered_v :=
                            signed(resize(level_ones_next, 12)) -
                            to_signed(LEVEL_CENTER, 12);

                        if level_centered_v(level_centered_v'high) = '1' then
                            level_magnitude_v := unsigned(-level_centered_v);
                        else
                            level_magnitude_v := unsigned(level_centered_v);
                        end if;

                        -- Scale magnitude for display and software use.
                        level_reg <= shift_left(resize(level_magnitude_v, 16), 4);

                        level_cnt      <= (others => '0');
                        level_ones_cnt <= (others => '0');

                    else
                        level_cnt      <= level_cnt + 1;
                        level_ones_cnt <= level_ones_next;
                    end if;

                    --------------------------------------------------------
                    -- New simple PDM to PCM path
                    --------------------------------------------------------
                    pcm_ones_next := pcm_ones_cnt;

                    if mic_data = '1' then
                        pcm_ones_next := pcm_ones_cnt + 1;
                    end if;

                    if pcm_cnt = PCM_DECIM_MAX then

                        -- Center around 78 ones per 156 PDM bits.
                        -- Result range is roughly -78 to +78.
                        pcm_centered_v :=
                            signed(resize(pcm_ones_next, 10)) -
                            to_signed(PCM_CENTER, 10);

                        -- Scale to signed 16 bit range.
                        -- This is not HiFi audio. It is enough for lightweight
                        -- acoustic features on MicroBlaze.
                        pcm_scaled_v := shift_left(resize(pcm_centered_v, 16), PCM_SHIFT);

                        pcm_sample_reg <= pcm_scaled_v;

                        -- Stretch valid for GPIO debug.
                        valid_stretch_cnt <= VALID_STRETCH_MAX;
                        pcm_valid_reg <= '1';

                        pcm_cnt      <= (others => '0');
                        pcm_ones_cnt <= (others => '0');

                    else
                        pcm_cnt      <= pcm_cnt + 1;
                        pcm_ones_cnt <= pcm_ones_next;
                    end if;

                end if;
            end if;
        end if;
    end process;

end rtl;