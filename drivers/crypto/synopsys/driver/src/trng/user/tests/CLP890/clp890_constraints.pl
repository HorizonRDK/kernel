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

clp890_constraints.pl [-d device]

checks that all constraints are implemented properly

=cut

use warnings;
use strict;
use Testlib;

##### Variables

print "/*************************************/\n";
print "/********** Constraints Test *********/\n";
print "/*************************************/\n";

##### Reload
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail("failed to reload the module\n"); }

##### init
Testlib::init();
Testlib::sysfs_open();


pred_resist_check();
sec_strength_check();
cmd_seq_check();

Testlib::pass();

########## Command sequence check
# checks:
# some sequences should and some should not go through. If received an error, it should be recoverable.
sub cmd_seq_check {
   print "==================== Command Sequence Check\n";
   
   # Step 1: uninstantiate twice. The bare code will return a flag saying that there is no instance, but, it can't
   # be seen in the kernel code, since it's not an actual error.
   my $step = 1;
   Testlib::set_secure("on");
   Testlib::do_uninstantiate($step);
   Testlib::do_uninstantiate($step);
   
   # Step 2: Instantiate with sec_strength=128 and no pred_resist and no ps
   # then instantiate again multiple times. it should return error each time.
   $step++;
   Testlib::do_instantiate($step, 128, 0, 0);
   for (my $i = 0; $i < Testlib::rand_num(); $i++) {
      Testlib::do_instantiate($step, 128, 0, 1);
   }
   
   # Step 3: uninstantiate and reseed. Reseed should return error
   $step++;
   Testlib::do_uninstantiate($step);
   Testlib::do_reseed($step, 0, 128, 1);
   
   # Step 4: instantiate and then do reseed+generate multiple times
   $step++;
   Testlib::do_instantiate($step, 128, 0, 0);
   for (my $i = 0; $i < (int(rand(5)) + 1); $i++) {
      Testlib::do_reseed($step, 0, 0);
      Testlib::do_generate($step, 128, 0, 0);
   }
   
   # Step 5: uninstantiate and generate. should return error. Then instantaite and generate.
   $step++;
   Testlib::do_uninstantiate($step);
   Testlib::do_generate($step, 128, 0, 1);
   Testlib::do_instantiate($step, 128, 0, 0);
   Testlib::do_generate($step, 128, 0, 0);
   
}


########## Prediction resistance check
# checks:
# 1. once pred_resist is on, SEEDx has to change after each generate.
# 2. pred_resist can change from 1 to 0 once the DRBG is instantiated.
# 3. pred_resist cannot change from 0 to 1 once the DRBG is instantiated.
sub pred_resist_check {   
   my $sec_strength = 128;

   print "==================== Prediction Resistance Check\n";
   
   # Initialize: uninstantiate and put the code in the non-secure mode
   Testlib::do_uninstantiate("Initialize");
   Testlib::set_secure("off");
   
   # Step 1: instantiate with pred_resist on
   Testlib::do_instantiate(1, $sec_strength, 1, 0);
   
   # Step 2: read SEEDx and store
   my $prev_seed_reg = Testlib::get_seed_reg();
   
   # Step 3: do generate random number of times and make sure SEED register changes every time
   my $rpt = int(rand(5)) + 1;
   for (my $i = 0; $i < $rpt; $i++) {
      Testlib::do_generate(3, $sec_strength, 1, 0, 1);
      my $seed_reg = Testlib::get_seed_reg();
      if ($seed_reg eq $prev_seed_reg) {
         Testlib::fail("Step 3, repeat $i: Seed register did not change.\n");
      }
      $prev_seed_reg = $seed_reg;
   }
   
   # Step 4: same as Step 3, but without pred_resist. SEED register must stay the same
   $prev_seed_reg = Testlib::get_seed_reg();
   $rpt = int(rand(5)) + 1;
   for (my $i = 0; $i < $rpt; $i++) {
      Testlib::do_generate(4, $sec_strength, 0, 0, 1);
      my $seed_reg = Testlib::get_seed_reg();
      if ($seed_reg ne $prev_seed_reg) {
         Testlib::fail("Step 4, repeat $i: Seed register is changed.\n");
      }      
   }
}

########## Security Strength check
# checks:
# 1. security strength input can only be an integer in (0,256]
# 2. security strength can only change to a higher value before the DRBG is instantiated
# 3. security strength can change to a lower value when the DRBG is already instantiated
# 4. when security strength chagnes to lower value when the DRBG is instantiated, 
sub sec_strength_check {     
   my $ret;
   my $step;
   my $pred_resist;
   my $sec_strength;
   my %features;
   
   print "==================== Security Strength Check\n";
   
   # Initialize: uninstantiate
   Testlib::set_secure("on");
   Testlib::do_uninstantiate("Initialize");
   
   # Step 1: set sec_strength should not get set to 0, random negative value, 257 and random positive larger than 256
   $step = 1;   
   # sec_strength = 0
   $sec_strength = 0;
   if (($ret = Testlib::set_sec_strength($sec_strength)) > 0) {
      Testlib::fail("Step $step: sec_strength got set to $sec_strength ($ret)\n");
   }
   # -256 <= sec_strength <= -1
   $sec_strength = -1*(int(rand(256))+1);
   if (($ret = Testlib::set_sec_strength($sec_strength)) > 0) {
      Testlib::fail("Step $step: sec_strength got set to $sec_strength ($ret)\n");
   }
   # sec_strength = 257
   $sec_strength = 257;
   if (($ret = Testlib::set_sec_strength($sec_strength)) > 0) {
      Testlib::fail("Step $step: sec_strength got set to $sec_strength ($ret)\n");
   }
   # 258 <= sec_strength < 2**32
   $sec_strength = int(rand(2**32-258))+258;
   if (($ret = Testlib::set_sec_strength($sec_strength)) > 0) {
      Testlib::fail("Step $step: sec_strength got set to $sec_strength ($ret)\n");
   }
   
   # Step 2: make sure sec_strength can do both 0->1 and 1->0 when DRBG is not instantiated 
   # only if the security strength can get set to 1 (i.e. AES256 is instantiated)
   $ret = Testlib::get_clp890_features();
   %features = %{$ret};

   if ($features{drbg_arch}) {
      $step = 2;
      # 1 <= sec_strength <= 128   
      $sec_strength = int(rand(128))+1;
      if (($ret = Testlib::set_sec_strength($sec_strength)) <= 0) {
         Testlib::fail("Step $step: sec_strength could not set to $sec_strength ($ret)\n");
      }
   
      # 129 <= sec_strength <= 256   
      $sec_strength = int(rand(128))+129;
      if (($ret = Testlib::set_sec_strength($sec_strength)) <= 0) {
         Testlib::fail("Step $step: sec_strength could not set to $sec_strength ($ret)\n");
      }
      # going down again, 1 <= sec_strength <= 128   
      $sec_strength = int(rand(128))+1;
      if (($ret = Testlib::set_sec_strength($sec_strength)) <= 0) {
         Testlib::fail("Step $step: sec_strength could not set to $sec_strength ($ret)\n");
      }
      # up again, 129 <= sec_strength <= 256   
      $sec_strength = int(rand(128))+129;
      if (($ret = Testlib::set_sec_strength($sec_strength)) <= 0) {
         Testlib::fail("Step $step: sec_strength could not set to $sec_strength ($ret)\n");
      }
   }
   
   # Step 3: instantiate the DRBG with the highest possible security strength
   if ($features{drbg_arch}) {
      $sec_strength = 256;
   } else {
      $sec_strength = 128;
   }
   Testlib::do_instantiate(3, $sec_strength, ($pred_resist = int(rand(2))), 0);
   
   # Step 4: security strength can chagne to a lower value
   Testlib::do_generate(4, 128, $pred_resist, 0);
   
   # Step 5: security strength cannot chagne to a higher value
   if ($features{drbg_arch}) {
      Testlib::do_generate(5, 256, $pred_resist, 1);     
   }
}

