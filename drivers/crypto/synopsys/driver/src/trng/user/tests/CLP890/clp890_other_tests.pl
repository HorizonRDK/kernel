#!/usr/bin/perl
# ------------------------------------------------------------------------
#
#                (C) COPYRIGHT 2012 - 2016 SYNOPSYS, INC.
#                          ALL RIGHTS RESERVED
#
#  (C) COPYRIGHT 2012-2016 Synopsys, Inc. 
#  This Synopsys software and all associated documentation are 
#  proprietary to Synopsys, Inc. and may only be used pursuant 
#  to the terms and conditions of a written license agreement 
#  with Synopsys, Inc. All other use, reproduction, modification, 
#  or distribution of the Synopsys software or the associated 
#  documentation is strictly prohibited.
#
# ------------------------------------------------------------------------

=head1 SYNOPSIS

clp890_other_tests.pl [-d device]

Any test that is not covered by other scripts:
   1. clp890_uninstantiate:
      verifies the uninstantiate function
   2. clp890_kat:
      verifies the Known Answer Test API

=cut

use warnings;
use strict;
use Testlib;

##### Variables

print "/*************************************/\n";
print "/************* Other Test ************/\n";
print "/*************************************/\n";

##### Reload
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail("failed to reload the module\n"); }

##### init
Testlib::init();
Testlib::sysfs_open();

clp890_uninstantiate();
clp890_kat();

Testlib::pass();

########## Uninstantiate API check
# checks:
# 1. call when no instantiated, make sure everything gets zeroized. Cannot verify the return flag
#    saying that there is no state.
# 2. call when DRBG is instantaited, make sure everything gets zerozied and the DRBG state is also removed.
# 3. make sure we can zeroize after generate and reseed
sub clp890_uninstantiate {
   my $ret;
   my %features;   

   print "==================== Uninstantiate check\n";
   
   my $a512bit_val  = "6545c0529d372443b392ceb3ae3a99a30f963eaf313280f1d1a1e87f9db373d361e75d18018266499cccd64d9bbb8de0185f213383080faddec46bae1f784e5a";   
   my $a384bit_val  = "df5d73faa468649edda33b5cca79b0b05600419ccb7a879ddfec9db32ee494e5531b51de16a30f769262474c73bec010";   

   $ret = Testlib::get_clp890_features();
   %features = %{$ret};
   
   # step 1: put the core in non-secure mode to be able to see internal secret info.
   #         load something into the SEED and NPA_DATA registers.
   #         uninstantiate, make sure everything gets zeroized.   
   my $step = 1;
   Testlib::do_uninstantiate($step); # uninstantaite first to make sure there is no instance.
   Testlib::set_secure("off");
   my $sec_strength = (int(rand(2)) && $features{drbg_arch}) ? 256 : 128;
   Testlib::set_sec_strength($sec_strength);
   Testlib::write_seed_reg($a384bit_val); # write SEED register
   Testlib::write_npa_reg($a512bit_val); # write NPA_DATA register
   # uninstantiate
   Testlib::do_uninstantiate($step);
   is_zeroized($step);
   
   # step 2: instantiate, make sure SEED, NPA and state are not zero.
   #         uninstantiate and make sure zeroize happens correctly.
   $sec_strength = (int(rand(2)) && $features{drbg_arch}) ? 256 : 128;
   my $pred_resist = int(rand(2));
   Testlib::do_instantiate($step, $sec_strength, $pred_resist, 0);
   not_zeroized($step);
   # uninstantiate
   Testlib::do_uninstantiate($step);
   is_zeroized($step);
   
   # Step 3: insantiate and generate and uninstantiate and check   
   Testlib::do_uninstantiate($step);
   $sec_strength = (int(rand(2)) && $features{drbg_arch}) ? 256 : 128;
   $pred_resist = int(rand(2));
   Testlib::do_instantiate($step, $sec_strength, $pred_resist, 0);      
   Testlib::do_generate($step, $sec_strength, $pred_resist, 0);
   not_zeroized($step);   
   # uninstantiate
   Testlib::do_uninstantiate($step);
   is_zeroized($step);
   
   # Step 4: instantiate and generate and reseed and uninstantiate and check
   Testlib::do_uninstantiate($step);
   $sec_strength =(int(rand(2)) && $features{drbg_arch}) ? 256 : 128;
   $pred_resist = int(rand(2));
   Testlib::do_instantiate($step, $sec_strength, $pred_resist, 0);      
   Testlib::do_generate($step, $sec_strength, $pred_resist, 0);
   Testlib::do_reseed($step, $pred_resist, 0);
   not_zeroized($step);   
   # uninstantiate
   Testlib::do_uninstantiate($step);
   is_zeroized($step);   
}

sub is_zeroized {
   my ($step) = @_;      
   
   my $a512bit_zero = "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";   
   my $a384bit_zero = "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
   my $a256bit_zero = "0000000000000000000000000000000000000000000000000000000000000000";
   my $a128bit_zero = "00000000000000000000000000000000";
   
   print ">> Step $step: check that zeroized has earased internal data\n";

   my $seed_reg = Testlib::get_seed_reg();
   my $npa_reg = Testlib::get_npa_reg();
   my ($state_key, $state_v) = Testlib::get_hw_state();
   if ($seed_reg ne $a384bit_zero) { Testlib::fail(">> Step $step: Seed register did not get zeroized\n"); }
   if ($npa_reg ne $a512bit_zero) { Testlib::fail(">> Step $step: NPA_DATA register did not get zeroized\n"); }
   if ($state_key ne $a256bit_zero) { Testlib::fail(">> Step $step: HW_STATE:Key did not get zeroized ($state_key)\n"); }
   if ($state_v ne $a128bit_zero) { Testlib::fail(">> Step $step: HW_STATE:V did not get zeroized ($state_v)\n"); }
}

sub not_zeroized {
   my ($step) = @_;      
   
   my $a512bit_zero = "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";   
   my $a384bit_zero = "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
   my $a256bit_zero = "0000000000000000000000000000000000000000000000000000000000000000";
   my $a128bit_zero = "00000000000000000000000000000000";
   
   print ">> Step $step: check that registers are not zeroized\n";
   
   my $seed_reg = Testlib::get_seed_reg();
   my $npa_reg = Testlib::get_npa_reg();
   my ($state_key, $state_v) = Testlib::get_hw_state();
   if ($seed_reg eq $a384bit_zero) { Testlib::fail(">> Step $step: Seed register is zero\n"); }
   if ($npa_reg eq $a512bit_zero) { Testlib::fail(">> Step $step: NPA_DATA register is zero\n"); }
   if ($state_key eq $a256bit_zero) { Testlib::fail(">> Step $step: HW_STATE:Key is zero\n"); }
   if ($state_v eq $a128bit_zero) { Testlib::fail(">> Step $step: HW_STATE:V is zero\n"); }
}

########## KAT API check
# call the API with "full" and all four options. Also with an invalid optino to make sure full goes through.
sub clp890_kat {
   print "==================== KAT check\n";
   
   Testlib::write_kat("full");
   Testlib::write_kat("00");
   Testlib::write_kat("01");
   Testlib::write_kat("10");
   Testlib::write_kat("11");
   Testlib::write_kat("invalid");
}

