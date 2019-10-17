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

clp890_up_n_running.pl [-d device]

Tests the normal mode of operation of the CLP890 right after the 
module is loaded.

=cut

use warnings;
use strict;
use Testlib;

##### Variables

print "/*************************************/\n";
print "/******** Up and Running Test ********/\n";
print "/*************************************/\n";

##### Reload
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail("failed to reload the module\n"); }

##### init
Testlib::init();
Testlib::sysfs_open();

##### open attributes
my $rd_sec_strength = Testlib::open_attribute($Testlib::attr_sec_strength, "<");
my $rd_secure = Testlib::open_attribute($Testlib::attr_secure, "<");

##### get CLP890 features
my $state_ref = Testlib::get_clp890_features();
my %state = %{$state_ref};
print "CLP890 Features: DRBG_ARCH = $state{drbg_arch}, SECURE_RESET_STATE = $state{sec_rst_state}, EXTRA_PS_PRESENT = $state{extra_ps_present}\n";

##### Step 1: get the sec_strength that the DRBG is instantiated with
my $sec_strength = Testlib::read_attribute($rd_sec_strength);
if (($sec_strength != 128) && ($sec_strength != 256)) {
   Testlib::fail("Invalid security strength ($sec_strength)\n");
}

##### Step 2: Check initial value of pred_resist and secure
my $pred_resist = Testlib::get_hw_pred_resist();
my $secure = (Testlib::read_attribute($rd_secure) eq "on") ? 1 : 0;
if (!$pred_resist) { Testlib::fail("Core is not initialized to prediction resistance turned on\n"); }
if (!$secure) { Testlib::fail("Core is not initialized to the secure mode\n"); }

print "Initial state: security strength = $sec_strength | prediction resistence = $pred_resist | secure mode = $secure\n";

##### Step 3: Generate rand times, sec_strength=current and pred_resist=1. 
#             RANDx should change, there should not be any error.
Testlib::do_generate(3, $sec_strength, 1, 0);

##### Step 4: Generate rand times, sec_strength=current and pred_resist=0. 
#             RANDx should change, there should not be any error.
Testlib::do_generate(4, $sec_strength, 0, 0);

##### Step 5: Generate rand times, sec_strength=current and pred_resist=1. Must return error, since pred_resist cannot change back to 1.
#             Call Generate rand times, again, with pred_resist=0 to make sure that we can recover from the previous errors. RANDx should change and no errors.
Testlib::do_generate("5-0", $sec_strength, 1, 1);

# now generate again to make sure we can recover from errors
Testlib::do_generate("5-1", $sec_strength, 0, 0);

##### Step 6: if possible call Generate with lower security strength. 
#             RANDx must change, no errors
if ($sec_strength == 256) {
   $sec_strength = 128;
   Testlib::do_generate(6, $sec_strength, 0, 0);
}

##### Step 7: if possible call Generate with higher security strength. 
#             must return error and recover
if ($sec_strength == 128) {
   $sec_strength = 256;
   Testlib::do_generate(7, $sec_strength, 0, 1);
   
   # recover
   $sec_strength = 128;
   Testlib::do_generate(7, $sec_strength, 0, 0);
}

##### Step 8: call Reseed, no error
Testlib::do_reseed(8, 0, 0);

##### Step 9: Generate rand times. securit strength = $sec_strength, pred_resist = 0
#             RANDx should change, no error
Testlib::do_generate(9, $sec_strength, 0, 0);

##### Step 10: Uninstantiate
Testlib::do_uninstantiate(10);

##### Step 11: Instantiate
if ($state{drbg_arch}) {
   $sec_strength = 256;
} else {
   $sec_strength = 128;
}
Testlib::do_instantiate(11, $sec_strength, 1, 0);

##### Step 12: Generate rand times. securit strength = $sec_strength, pred_resist = 1
#             RANDx should change, no error
Testlib::do_generate(12, $sec_strength, 1, 0);

Testlib::pass();

