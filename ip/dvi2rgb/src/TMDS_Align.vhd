-------------------------------------------------------------------------------
--
-- File: TMDS_Align.vhd
-- Author: Elod Gyorgy
-- Modified: George Castillo
-- Original Project: HDMI input on 7-series Xilinx FPGA
-- Date: 8 October 2014
--
-- Modified from TMDS_Decoder originally written by Elod Gyorgy, and renamed
-- to reflect the more limited use as a generator of aligned 10-bit symbols.
--
-------------------------------------------------------------------------------
-- (c) 2014 Copyright Digilent Incorporated
-- All Rights Reserved
--
-- This program is free software; distributed under the terms of BSD 3-clause
-- license ("Revised BSD License", "New BSD License", or "Modified BSD License")
--
-- Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation
--    and/or other materials provided with the distribution.
-- 3. Neither the name(s) of the above-listed copyright holder(s) nor the names
--    of its contributors may be used to endorse or promote products derived
--    from this software without specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
-- SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
-- CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
-- OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
-- OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-------------------------------------------------------------------------------
--
-- Purpose:
-- This module connects to one TMDS data channel and decodes TMDS data
-- according to DVI specifications. It phase aligns the data channel,
-- deserializes the stream, eliminates skew between data channels and decodes
-- data in the end.
-- sDataIn_p/n -> buffer -> de-serialize -> channel de-skew -> decode -> pData
--
-- Edit:
-- Originally, this module decoded TMDS data into DVI data, which is insufficient
-- for HDMI processing, which requires the 10-bit raw characters and more effort
-- to decode the packet type.  This module has been modified to contain 10-bit
-- symbols as outputs and also been renamed to reflect the fact that it is largely
-- just being used as an aligner, with TMDS decoding being done elsewhere.
-------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use work.DVI_Constants.ALL;

entity TMDS_Align is
    generic (
        -- how many subsequent control tokens make a valid blank detection
        kCtlTknCount        : natural := 128;
        -- what is the maximum time interval for a blank to be detected
        kTimeoutMs          : natural := 50;
        -- what is the RefClk frequency
        kRefClkFrqMHz       : natural := 200;
         -- delay in ps per tap
        kIDLY_TapValuePs    : natural := 78;
         -- number of bits for IDELAYE2 tap counter
        kIDLY_TapWidth      : natural := 5
    );
    port (
        --Recovered TMDS clock (CLKDIV)
        PixelClk            : in  std_logic;
        -- Recovered TMDS clock x5 (CLK)
        SerialClk           : in  std_logic;
        -- 200MHz reference clock
        RefClk              : in  std_logic;
       -- Asynchronous reset; must be reset when PixelClk/SerialClk is not within spec
        aRst                : in  std_logic;

        -- Serial TMDS data channel
        sDataIn             : in  std_logic;

        -- These signals may only be meaningful for DVI data - there is some
        -- decoding already done in this module
        pDataIn             : out std_logic_vector(7 downto 0);
        pC0                 : out std_logic;
        pC1                 : out std_logic;
        pVde                : out std_logic;
        -- Aligned 10-bit characters with no data or control signal decoding
        pDataIn10b          : out std_logic_vector(9 downto 0);

        -- Channel bonding (three data channels in total)
        pOtherChVld         : in  std_logic_vector(1 downto 0);
        pOtherChRdy         : in  std_logic_vector(1 downto 0);
        pMeVld              : out std_logic;
        pMeRdy              : out std_logic;

        -- Synchronous reset to restart lock procedure
        pRst                : in  std_logic;
        dbg_pAlignErr       : out std_logic;
        dbg_pEyeSize        : out STD_LOGIC_VECTOR(kIDLY_TapWidth-1 downto 0);
        dbg_pBitslip        : out std_logic
    );

end TMDS_Align;

architecture Behavioral of TMDS_Align is

    -- Three-period delay after bitslip
    constant kBitslipDelay  : natural := 3;
    -- Timeout Counter End
    constant kTimeoutEnd    : natural := kTimeoutMs * 1000 * kRefClkFrqMHz;

    signal pAlignRst        : std_logic;
    signal pLockLostRst_n   : std_logic;
    signal pBitslipCnt      : natural range 0 to kBitslipDelay - 1 := kBitslipDelay - 1;
    signal pDataIn8b        : std_logic_vector(7 downto 0);
    -- 10 bit deserialized data prior to lane alignment and deskewing
    signal pDataInRaw       : std_logic_vector(9 downto 0);
    -- 10 bit deserialized data after lane alignment and deskewing
    signal pDataInBnd       : std_logic_vector(9 downto 0);

    signal pMeRdy_int       : std_logic;
    signal pAligned         : std_logic;
    signal pAlignErr_int    : std_logic;
    signal pAlignErr_q      : std_logic;
    signal pBitslip         : std_logic;

    signal pIDLY_LD         : std_logic;
    signal pIDLY_CE         : std_logic;
    signal pIDLY_INC        : std_logic;
    signal pIDLY_CNT        : std_logic_vector(kIDLY_TapWidth - 1 downto 0);

    signal rTimeoutCnt      : natural range 0 to kTimeoutEnd-1;
    signal pTimeoutRst      : std_logic;
    signal pTimeoutOvf      : std_logic;
    signal rTimeoutRst      : std_logic;
    signal rTimeoutOvf      : std_logic;

    attribute MARK_DEBUG    : string;
    -- These are legacy DVI signals that have some degree of decoding performed
    -- prior to connecting up to the next layer in data handling (I think)
    attribute MARK_DEBUG of pDataIn         : signal is "TRUE";
    attribute MARK_DEBUG of pC0             : signal is "TRUE";
    attribute MARK_DEBUG of pC1             : signal is "TRUE";
    attribute MARK_DEBUG of pVde            : signal is "TRUE";
    -- If the hardware is recognized as an HDMI sink these should be 10-bit
    -- aligned characters without any additional decoding performed
    attribute MARK_DEBUG of pDataIn10b      : signal is "TRUE";

begin

    dbg_pAlignErr           <= pAlignErr_int;
    dbg_pBitslip            <= pBitslip;

    -- Deserialization block
    InputSERDES_X: entity work.InputSERDES
       generic map (
          kIDLY_TapWidth => kIDLY_TapWidth,
          kParallelWidth => 10
          )
       port map (
          PixelClk      => PixelClk,    -- in  std_logic
          SerialClk     => SerialClk,   -- in  std_logic
          sDataIn       => sDataIn,     -- in  std_logic
          pDataIn       => pDataInRaw,  -- out std_logic_vector(kParallelWidth-1 downto 0)
          pBitslip      => pBitslip,    -- in  std_logic
          pIDLY_LD      => pIDLY_LD,    -- in  std_logic
          pIDLY_CE      => pIDLY_CE,    -- in  std_logic
          pIDLY_INC     => pIDLY_INC,   -- in  std_logic
          pIDLY_CNT     => pIDLY_CNT,   -- out std_logic_vector(kIDLY_TapWidth-1 downto 0)
          aRst          => aRst         -- in  std_logic
       );

    -- reset min two period (ISERDESE2 requirement)
    -- de-assert synchronously with CLKDIV, min two period (ISERDESE2 requirement)
    
    --The timeout counter runs on RefClk, because it's a fixed frequency we can measure timeout
    --independently of the TMDS Clk
    --The xTimeoutRst and xTimeoutOvf signals need to be synchronized back-and-forth
    TimeoutCounter: process(RefClk)
    begin
        if Rising_Edge(RefClk) then
            if (rTimeoutRst = '1') then
                rTimeoutCnt     <= 0;
            elsif (rTimeoutOvf = '0') then
                rTimeoutCnt     <= rTimeoutCnt + 1;
            end if;
        end if;
    end process TimeoutCounter;

    rTimeoutOvf <= '0' when rTimeoutCnt /= kTimeoutEnd - 1 else '1';

    SyncBaseOvf: entity work.SyncBase
        generic map (
            kResetTo => '0',
            kStages => 2
        )
        port map (
            aReset      => aRst,
            InClk       => RefClk,
            iIn         => rTimeoutOvf,
            OutClk      => PixelClk,
            oOut        => pTimeoutOvf
        );

    SyncBaseRst: entity work.SyncBase
        generic map (
            kResetTo    => '1',
            kStages     => 2
        )
        port map (
            aReset      => aRst,
            InClk       => PixelClk,
            iIn         => pTimeoutRst,
            OutClk      => RefClk,
            oOut        => rTimeoutRst
        );

    -- Phase alignment controller to lock onto data stream
    PhaseAlignX: entity work.PhaseAlign
        generic map (
            kUseFastAlgorithm     => false,
            kCtlTknCount          => kCtlTknCount,
            kIDLY_TapValuePs      => kIDLY_TapValuePs,
            kIDLY_TapWidth        => kIDLY_TapWidth
        )
        port map (
            pRst                  => pAlignRst,
            PixelClk              => PixelClk,
            pTimeoutOvf           => pTimeoutOvf,
            pTimeoutRst           => pTimeoutRst,
            pData                 => pDataInRaw,
            pIDLY_CE              => pIDLY_CE,
            pIDLY_INC             => pIDLY_INC,
            pIDLY_CNT             => pIDLY_CNT,
            pIDLY_LD              => pIDLY_LD,
            pAligned              => pAligned,
            pError                => pAlignErr_int,
            pEyeSize              => dbg_pEyeSize
        );

    pMeVld <= pAligned;

    -- Bitslip when phase alignment exhausted the whole tap range and still no lock
    Bitslip: process(PixelClk)
    begin
        if Rising_Edge(PixelClk) then
            pAlignErr_q     <= pAlignErr_int;
            pBitslip        <= not pAlignErr_q and pAlignErr_int; -- single pulse bitslip on failed alignment attempt
        end if;
    end process Bitslip;

    ResetAlignment: process(PixelClk, aRst)
    begin
        if (aRst = '1') then
            pAlignRst       <= '1';
        elsif Rising_Edge(PixelClk) then
            if (pRst = '1' or pBitslip = '1') then
                pAlignRst   <= '1';
            elsif (pBitslipCnt = 0) then
                pAlignRst   <= '0';
            end if;
        end if;
    end process ResetAlignment;

    -- Reset phase aligment module after bitslip + 3 CLKDIV cycles (ISERDESE2 requirement)
    BitslipDelay: process(PixelClk)
    begin
        if Rising_Edge(PixelClk) then
            if (pBitslip = '1') then
                pBitslipCnt     <= kBitslipDelay - 1;
            elsif (pBitslipCnt /= 0) then
                pBitslipCnt     <= pBitslipCnt - 1;
            end if;
        end if;
    end process BitslipDelay;

    -- Deserialized 10-bit data from the ISERDES and lane alignment information
    -- are sent here for deskewing and alignment between each instance of the
    -- TMDS decoder
    ChannelBondX: entity work.ChannelBond
        port map (
            PixelClk        => PixelClk,     -- in  std_logic
            pDataInRaw      => pDataInRaw,   -- in  std_logic_vector(9 downto 0)
            pMeVld          => pAligned,     -- in  std_logic
            pOtherChVld     => pOtherChVld,  -- in  std_logic_vector(1 downto 0)
            pOtherChRdy     => pOtherChRdy,  -- in  std_logic_vector(1 downto 0)
            pDataInBnd      => pDataInBnd,   -- out std_logic_vector(9 downto 0)
            pMeRdy          => pMeRdy_int    -- out std_logic
        );

    pMeRdy      <= pMeRdy_int;
    pDataIn10b  <= pDataInBnd;

    -- Below performs the 10B-8B decoding function
    -- DVI Specification: Section 3.3.3, Figure 3-6, page 31.
    pDataIn8b   <= pDataInBnd(7 downto 0) when pDataInBnd(9) = '0' else
                        not pDataInBnd(7 downto 0);

    TMDS_Decode: process (PixelClk)
    begin
        if Rising_Edge(PixelClk) then
            if (pMeRdy_int = '1' and pOtherChRdy = "11") then
                pDataIn <= x"00"; --added for VGA-compatibility (blank pixel needed during blanking)

                case (pDataInBnd) is
                    --Control tokens decode straight to C0, C1 values
                    when kCtlTkn0 =>
                        pC0         <= '0';
                        pC1         <= '0';
                        pVde        <= '0';
                    when kCtlTkn1 =>
                        pC0         <= '1';
                        pC1         <= '0';
                        pVde        <= '0';
                    when kCtlTkn2 =>
                        pC0         <= '0';
                        pC1         <= '1';
                        pVde        <= '0';
                    when kCtlTkn3 =>
                        pC0         <= '1';
                        pC1         <= '1';
                        pVde        <= '0';
                    --If not control token, it's encoded data
                    when others =>
                        pVde        <= '1';
                        pDataIn(0)  <= pDataIn8b(0);
                        for iBit in 1 to 7 loop
                            if (pDataInBnd(8) = '1') then
                                 pDataIn(iBit) <= pDataIn8b(iBit) xor pDataIn8b(iBit-1);
                            else
                                 pDataIn(iBit) <= pDataIn8b(iBit) xnor pDataIn8b(iBit-1);
                            end if;
                        end loop;
                end case;
            else --if we are not aligned on all channels, gate outputs
                pC0 <= '0';
                pC1 <= '0';
                pVde <= '0';
                pDataIn <= x"00";
            end if;
        end if;
    end process;

end Behavioral;
