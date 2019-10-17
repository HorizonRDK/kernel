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

clp890_reminders.pl [-d device]

verifies all CLP890 reminders

=cut

use warnings;
use strict;
use Testlib;

##### Variables
my $CLP890_DFLT_MAX_BITS_PER_REQ = 1<<19;
my $CLP890_DFLT_MAX_REQ_PER_SEED = 1<<7;

my $sec_strength;

print "/*************************************/\n";
print "/*********** Reminders Test **********/\n";
print "/*************************************/\n";

##### Reload
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail("failed to reload the module\n"); }

##### init
Testlib::init();
Testlib::sysfs_open();

# Step 1: check default values
# reload to set reminders back to default
system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;");
my $step = 1;
my $max_bits_p_req = hex Testlib::get_max_bits_p_req();
my $max_req_p_seed = hex Testlib::get_max_req_p_seed();
if ($max_bits_p_req != $CLP890_DFLT_MAX_BITS_PER_REQ) {
   Testlib::fail("Step $step: max_bits_per_req read value does not match the default\n     read_value = $max_bits_p_req\n     default    = $CLP890_DFLT_MAX_BITS_PER_REQ\n");
}
if ($max_req_p_seed != $CLP890_DFLT_MAX_REQ_PER_SEED) {
   Testlib::fail("Step $step: max_req_per_seed read value does not match the default\n     read_value = $max_req_p_seed\n     default    = $CLP890_DFLT_MAX_REQ_PER_SEED\n");
}
print "Default values match!\n";

# Step 2: change max_req_per_seed and request more than that with and without pred_resis
# uninstantiate and change the reminder
$step++;
Testlib::set_secure("off");
Testlib::do_uninstantiate($step);
my $max = int(rand(5)+2);
if (Testlib::set_max_req_p_seed(sprintf("%016x", $max)) <= 0) { Testlib::fail("Set max_req_per_seed failed\n"); }
print ">> Step $step: max_req_per_seed set to $max\n";

# instantiate without pred_resist
Testlib::do_instantiate($step, ($sec_strength = (int(rand(1))+1)*128), 0, 0);

# generate more than max (any generate after must change SEED register)
my $prev_seed_reg = Testlib::get_seed_reg();
my $seed_reg;
for (my $i = 0; $i < ($max+1); $i++) {
   Testlib::do_generate($step, $sec_strength, 0, 0, 1);
   $seed_reg = Testlib::get_seed_reg();
   
   if ($i < $max) {
      if ($seed_reg ne $prev_seed_reg) {
         Testlib::fail("SEED register is changed in iteration $i\n");
      }
   } else {
      if ($seed_reg eq $prev_seed_reg) {
         Testlib::fail("SEED register has not changed in iteration $i\n");
      }
   }
}

# Step 3: change max_bits_per_req and request more and equal
$step++;

# reload to reset reminders
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail(">> Step $step: failed to reload the module\n"); }

# each dd command triggers the generate command only with 512 bit requests. So:
# For equal: first set the maximum to 512 and request.
# For more: set the max to a random value less than 512 and request.

# Equal: change max_bits_per_request to 512 and request
Testlib::do_uninstantiate($step);
if (Testlib::set_max_bits_p_req(sprintf("%08x", 512)) <= 0) { Testlib::fail(">> Step $step: failed to set max_bits_per_req\n"); }
Testlib::do_instantiate($step, 128, 0, 0);
if (system("dd if=/dev/hwrng ibs=64 count=1 of=/dev/random")) { Testlib::fail(">> Step $step: failed to run dd in stage 1\n"); }
print ">> Step $step: stage 1 passed\n";

# More: change max_bits_per_request to random less than 512 and request
Testlib::do_uninstantiate($step);
if (Testlib::set_max_bits_p_req(sprintf("%016x", int(rand(511)) + 1)) <= 0) { Testlib::fail(">> Step $step: failed to set max_bits_per_req\n"); }
Testlib::do_instantiate($step, 128, 0, 0);
if (!system("dd if=/dev/hwrng ibs=64 count=1 of=/dev/random")) { Testlib::fail(">> Step $step: failed to fail dd in stage 2\n"); }
print ">> Step $step: stage 2 passed\n";

Testlib::pass();